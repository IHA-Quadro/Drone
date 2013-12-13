#include "QueueList.h"

// init the queue (constructor).
template<typename T>
QueueList<T>::QueueList() 
{
	size = 0;       // set the size of queue to zero.
	head = NULL;    // set the head of the list to point nowhere.
	tail = NULL;    // set the tail of the list to point nowhere.
	printer = NULL; // set the printer of queue to point nowhere.
}

// clear the queue (destructor).
template<typename T>
QueueList<T>::~QueueList() 
{
	// deallocate memory space of each node in the list.
	for (link t = head; t != NULL; head = t) 
	{
		t = head->next; 
		delete head;
	}

	size = 0;       // set the size of queue to zero.
	tail = NULL;    // set the tail of the list to point nowhere.
	printer = NULL; // set the printer of queue to point nowhere.
}

// push an item to the queue.
template<typename T>
void QueueList<T>::push (const T i) 
{
	link t = tail;
	tail = (link) new node;

	if (tail == NULL)
		exit ("QUEUE: insufficient memory to create a new node.");

	tail->next = NULL;
	tail->item = i;

	if (isEmpty ())
		head = tail;
	else
		t->next = tail;

	size++;
}

template<typename T>
T QueueList<T>::PeekAverage() const
{
	T value = 0.0;
	link t = head;

	for(int i = 0; i < size; i++)
	{
		value += t->item;
		t = t->next;
	}

	return value / size;
}

template<typename T>
void QueueList<T>::EmptyList()
{
	while(!this->isEmpty())
		this->pop();
}

// pop an item from the queue.
template<typename T>
T QueueList<T>::pop () 
{
	if (isEmpty ())
		exit ("QUEUE: can't pop item from queue: queue is empty.");

	T item = head->item;
	link t = head->next; 
	delete head; 
	head = t;
	size--;

	return item;
}

// get an item from the queue.
template<typename T>
T QueueList<T>::peek () const 
{
	if (isEmpty ())
		exit ("QUEUE: can't peek item from queue: queue is empty.");

	return head->item;
}

// check if the queue is empty.
template<typename T>
bool QueueList<T>::isEmpty () const 
{
	return head == NULL;
}

// get the number of items in the queue.
template<typename T>
int QueueList<T>::count () const {

	return size;
}

// set the printer of the queue.
template<typename T>
void QueueList<T>::setPrinter (Print & p) 
{
	printer = &p;
}

// exit report method in case of error.
template<typename T>
void QueueList<T>::exit (const char * m) const 
{
	// print the message if there is a printer.
	if (printer)
		printer->println (m);

	// loop blinking until hardware reset.
	//blink ();
}

// led blinking method in case of error.
template<typename T>
void QueueList<T>::blink () const 
{
	pinMode (ledPin, OUTPUT);

	// continue looping until hardware reset.
	while (true) 
	{
		digitalWrite (ledPin, HIGH); // sets the LED on.
		delay (250);                 // pauses 1/4 of second.
		digitalWrite (ledPin, LOW);  // sets the LED off.
		delay (250);                 // pauses 1/4 of second.
	}
}

template<typename T>
int QueueList<T>::PeekLastElementFilter(int filterElement) const
{
	int value = filterElement; //Value to use
	link t = head; //last element

	if (isEmpty ())
		exit ("QUEUE: can't peek item from queue: queue is empty.");

	for(int i = 0; i < size; i++)
	{
		if(t != NULL)
		{
			if(t->item != filterElement)
				value = t->item;

			t = t->next;
		}
	}

	return value;
}

template class QueueList<float>;
//Begrundelse: 30/10-2013
// http://www.parashift.com/c++-faq-lite/separate-template-class-defn-from-decl.html