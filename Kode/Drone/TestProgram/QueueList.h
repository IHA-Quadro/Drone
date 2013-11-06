#ifndef _QUEUELIST_H
#define _QUEUELIST_H

#include <Arduino.h>

template<typename T>
class QueueList 
{
  public:
    QueueList();
    ~QueueList();
    void push (const T i);
    T pop ();
    T peek () const;
		void EmptyList();
    bool isEmpty () const;
    int count () const;
    void setPrinter (Print & p);

  private:
    void exit (const char * m) const;
    void blink () const;
    static const int ledPin = 13;

    typedef struct node {
      T item;      // the item in the node.
      node * next; // the next node in the list.
    } node;

    typedef node * link; // synonym for pointer to a node.



    Print * printer; // the printer of the queue.
    int size;        // the size of the queue.
    link head;       // the head of the list.
    link tail;       // the tail of the list.
};



#endif // _QUEUELIST_H