#include"hc_ioctl.h"

#define DRIVER_NAME "hc_driver"

// device configuration parameters
#define DEVICE_NAME "HCSR"
#define MAX_DEVICES 10
#define DEFAULT_DEVICES 2 			// NOTE: DEFAULT_DEVICES <= MAX_DEVICES
#define DEVICE_FIFO_SIZE 5			// size of the per device circular buffer
#define M_MIN 5						// minimum number of samples
#define DELTA_MIN 60				// sampling time in ms
#define TRIGGER_PIN_COUNT 22
#define ECHO_PIN_COUNT 14

// valid GPIO pins
const int trigger_pins[TRIGGER_PIN_COUNT] = {11, 12, 13, 61, 14, 62, 6, 0, 1, 38, 40, 4, 10, 5, 15, 7, 48, 50, 52, 54, 56, 58};
const int echo_pins[ECHO_PIN_COUNT] = {61, 62, 6, 0, 1, 4, 5, 7, 48, 50, 52, 54, 56, 58};

// device specific structure
typedef struct dev_struct{
	char device_name[20];
	int trigger_pin;
	int echo_pin;
	int m;
	int delta;
	int temp_measure;				// temporary storage of measurement
	unsigned long long t1, t2;		// temporary storage of echo rise and fall timestamps
	int state; 						// 0 - echo low to high 1 - echo high to low
	struct cbuff* cbuff;			// per device circular buffer
	struct semaphore sem; 			// represents an ongoing measurement
	spinlock_t mr_lock; 			// reprensents a trigger signal or an echo signal
	struct cdev cdev_struct;
} *dev_ptr;

// driver specific functions
static void device_setup(dev_ptr device, int index);
static void device_kill(dev_ptr device, int index);
static int validate_pins(int trig, int echo, dev_ptr device);
static int configure_pins(int trig, int echo, dev_ptr device);
static void measurement_routine(dev_ptr device);
static irqreturn_t irq_handler(int irq, void *device);


//*******************************circular buffer implementation***************************************
struct cbuff_node{
	struct cbuff_node *next_ptr;		// points to the next element in ring
	struct node *node_ptr;				// points to the data node
};

struct cbuff{
	struct cbuff_node *head;			// points to the node after the latest written node
	struct cbuff_node *tail;			// points to the node after the latest read node
};

struct cbuff *cbuff_init(int size);
int is_cbuff_full(struct cbuff *cb);
int is_cbuff_empty(struct cbuff *cb);
void cbuff_destroy(struct cbuff* cb);
void cbuff_put(struct cbuff *cb, struct node node);
struct node cbuff_get(struct cbuff *cb);
void cbuff_clear(struct cbuff *cb);

int cbuff_max_size;						// stores the size of the ring buffer

/* 
 * called to create a circular buffer
 * returns a pointer to the created circular buffer
 * @size - (size - 1) of the circular buffer required
 * -1 to accomodate the sacrifical node of a circular buffer
 */
struct cbuff *cbuff_init(int size){
	
	int i;
	struct cbuff *cbuff;
	struct cbuff_node *temp;

	cbuff_max_size = size;
	
	cbuff = kmalloc(sizeof(struct cbuff), GFP_KERNEL);
	cbuff->head = kmalloc(sizeof(struct cbuff_node), GFP_KERNEL);
	cbuff->head->node_ptr = kmalloc(sizeof(struct node), GFP_KERNEL);
	*cbuff->head->node_ptr = (struct node){-1, -1};
	// printk("%p %d\n", cbuff->head, cbuff->head->node_ptr->distance);
	cbuff->tail = cbuff->head;

	temp = cbuff->head;
	for(i = 0; i < cbuff_max_size; i++){
		if(i == cbuff_max_size - 1){
			temp->next_ptr = cbuff->head;
			// printk("%p\n", temp->next_ptr);
		}else{
			temp->next_ptr = kmalloc(sizeof(struct cbuff_node), GFP_KERNEL);
			temp->next_ptr->node_ptr = kmalloc(sizeof(struct node), GFP_KERNEL);
			*temp->next_ptr->node_ptr = (struct node){-1, -1};
			// printk("%p %d\n", temp->next_ptr, temp->next_ptr->node_ptr->distance);
			temp = temp->next_ptr;
		}
	}

	return cbuff;
}

/*
 * called to destroy a circular buffer
 * @cb - circular buffer to be destroyed
 */
void cbuff_destroy(struct cbuff *cb){
	int i;
	struct cbuff_node *temp1 = cb->head;
	struct cbuff_node *temp2 = cb->head->next_ptr;
	for(i = 0; i < cbuff_max_size; i++){
		// printk("%p %d&&&\n",temp1, temp1->node_ptr->distance);
		kfree(temp1->node_ptr);		
		kfree(temp1);
		temp1 = temp2;
		temp2 = temp2->next_ptr;
	}
	cb->head = temp2;
	cb->tail = temp2;
	kfree(cb);
}

/*
 * checks if a circular buffer is full
 * @cb - circular buffer to be checked
 * return 0 if false, 1 if true
 */
int is_cbuff_full(struct cbuff *cb){
	
	if(cb->tail == cb->head->next_ptr){
		return 1;
	}else{
		return 0;
	}
}

/*
 * checks if a circular buffer is empty
 * @cb - circular buffer to be checked
 * return 0 if false, 1 if true
 */
int is_cbuff_empty(struct cbuff *cb){
	if(cb->tail == cb->head){
		return 1;
	}else{
		return 0;
	}
}

/*
 * puts a node at the head position of circular buffer
 * @cb - circular buffer to be used
 * @node - node which needs to be inserted
 */
void cbuff_put(struct cbuff *cb, struct node node){
	*cb->head->node_ptr = node;
	
	if(is_cbuff_full(cb)){
		// printk("buffer full, tail shifted fwd\n");
		cb->head = cb->head->next_ptr;
		cb->tail = cb->tail->next_ptr;
	}else{
		cb->head = cb->head->next_ptr;
	}
}

/*
 * returns a node from the tail position of circular buffer
 * @cb - circular buffer to be used
 */
struct node cbuff_get(struct cbuff *cb){
	struct node ret;
	if(is_cbuff_empty(cb)){
		// printk("read on buffer empty\n");
		return (struct node){-1, -1};
	}else{
		ret = *cb->tail->node_ptr;
		*cb->tail->node_ptr = (struct node){-1, -1};
		cb->tail = cb->tail->next_ptr;
		return ret;
	}
}

/*
 * clears the value of nodes to default values
 * @cb - circular buffer to be cleared
 */
void cbuff_clear(struct cbuff *cb){
	if(is_cbuff_empty(cb)){
		return;
	}else{
		while(!is_cbuff_empty(cb)){
			*cb->tail->node_ptr = (struct node){-1, -1};
			cb->tail = cb->tail->next_ptr;
		}
	}
}
//****************************************************************************************************