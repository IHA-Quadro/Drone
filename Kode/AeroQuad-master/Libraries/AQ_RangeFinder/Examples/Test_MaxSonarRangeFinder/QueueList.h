/*
 *  QueueList.h
 *
 *  Library implementing a generic, dynamic queue (linked list version).
 *
 *  ---
 *
 *  Copyright (C) 2010  Efstathios Chatzikyriakidis (contact@efxa.org)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  Version 1.0
 *
 *    2010-09-28  Efstathios Chatzikyriakidis  <contact@efxa.org>
 *
 *      - added exit(), blink(): error reporting and handling methods.
 *
 *    2010-09-25  Alexander Brevig  <alexanderbrevig@gmail.com>
 *
 *      - added setPrinter(): indirectly reference a Serial object.
 *
 *    2010-09-20  Efstathios Chatzikyriakidis  <contact@efxa.org>
 *
 *      - initial release of the library.
 *
 *  ---
 *
 *  For the latest version see: http://www.arduino.cc/
 */


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