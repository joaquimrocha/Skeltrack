/*
 * pqueue.c
 *
 * Skeltrack - A Free Software skeleton tracking library
 * Copyright (C) 2012 Igalia S.L.
 *
 * Based on the implementation at:
 *  http://algs4.cs.princeton.edu/44sp/IndexMinPQ.java.html
 *
 * Authors:
 *  Iago López Galeiras <iaguis@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License at http://www.gnu.org/licenses/lgpl-3.0.txt
 * for more details.
 */

#include "pqueue.h"

PQueue *
pqueue_new (guint max_size, guint width, guint height)
{
  guint i;

  PQueue *queue = g_slice_new (PQueue);
  queue->elements = g_slice_alloc0 ((max_size + 1) * sizeof (PQelement));
  queue->map = g_slice_alloc (width * height * sizeof(guint));

  for (i=0; i<width*height; i++)
    queue->map[i] = -1;

  queue->size = 0;
  queue->max_size = max_size;
  queue->width = width;
  queue->height = height;
  return queue;
}

static void
swap (PQueue *queue, guint a, guint b)
{
  guint index_a, index_b;

  index_a = queue->elements[a].data->j * queue->width +
    queue->elements[a].data->i;
  index_b = queue->elements[b].data->j * queue->width +
    queue->elements[b].data->i;

  guint temp = queue->map[index_a];
  queue->map[index_a] = queue->map[index_b];
  queue->map[index_b] = temp;

  PQelement element_temp = queue->elements[b];
  queue->elements[b] = queue->elements[a];
  queue->elements[a] = element_temp;
}

static gboolean
greater (PQueue *queue, guint a, guint b)
{
  return queue->elements[a].priority > queue->elements[b].priority;
}

static void
swim (PQueue *queue, guint index)
{
  while (index > 1 && greater (queue, index/2, index))
    {
      swap (queue, index, index/2);
      index = index/2;
    }
}

static void
sink (PQueue *queue, guint index)
{
  guint j, size;
  size = queue->size;

  while (2 * index <= size)
    {
      j = 2 * index;

      if ((j < size) && (greater(queue, j, j+1)))
        j++;

      if (!greater (queue, index, j))
        break;

      swap (queue, index, j);

      index = j;
    }
}

void
pqueue_insert (PQueue *pqueue,
               Node *data,
               guint priority)
{
  guint index;

  pqueue->elements[++(pqueue->size)].data = data;
  pqueue->elements[pqueue->size].priority = priority;

  index = data->j * pqueue->width + data->i;
  pqueue->map[index] = pqueue->size;

  swim (pqueue, pqueue->size);
}

Node *
pqueue_pop_minimum (PQueue *pqueue)
{
  if (pqueue_is_empty (pqueue))
    return NULL;

  Node *minimum = pqueue->elements[1].data;
  guint index;
  swap (pqueue, 1, pqueue->size);
  pqueue->size--;

  index = minimum->j * pqueue->width + minimum->i;
  pqueue->map[index] = -1;

  sink (pqueue, 1);
  return minimum;
}

void
pqueue_delete (PQueue *pqueue,
               Node *data)
{
  guint index, pos;

  index = data->j * pqueue->width + data->i;
  pos = pqueue->map[index];

  Node *element = pqueue->elements[pos].data;

  swap (pqueue, pos, pqueue->size);
  pqueue->size--;

  pqueue->map[element->j * pqueue->width + element->i] = -1;

  sink (pqueue, pos);
}

gboolean
pqueue_has_element (PQueue *pqueue,
                    Node *data)
{
  return pqueue->map[data->j * pqueue->width + data->i] != -1;
}

gboolean
pqueue_is_empty (PQueue *pqueue)
{
  return pqueue->size == 0;
}

void
pqueue_free (PQueue *pqueue)
{
  g_slice_free1 ((pqueue->max_size + 1) * sizeof (PQelement), pqueue->elements);
  g_slice_free1 (pqueue->width * pqueue->height * sizeof (guint), pqueue->map);

  g_slice_free (PQueue, pqueue);
}

