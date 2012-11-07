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
pqueue_new (guint max_size)
{
  PQueue *queue = g_slice_new (PQueue);
  queue->elements = g_slice_alloc0 ((max_size + 1) * sizeof (PQelement));
  queue->map = g_hash_table_new (g_direct_hash, g_direct_equal);
  queue->size = 0;
  queue->max_size = max_size;
  return queue;
}

void swap (PQueue *queue, guint a, guint b)
{
  gpointer temp = g_hash_table_lookup (queue->map, queue->elements[a].data);
  g_hash_table_replace (queue->map,
                        queue->elements[a].data,
                        g_hash_table_lookup (queue->map,
                                             queue->elements[b].data));
  g_hash_table_replace (queue->map,
                        queue->elements[b].data,
                        temp);

  PQelement element_temp = queue->elements[b];
  queue->elements[b] = queue->elements[a];
  queue->elements[a] = element_temp;
}

gboolean greater (PQueue *queue, guint a, guint b)
{
  return queue->elements[a].priority > queue->elements[b].priority;
}

static void swim (PQueue *queue, guint index)
{
  while (index > 1 && greater (queue, index/2, index))
    {
      swap (queue, index, index/2);
      index = index/2;
    }
}

void sink (PQueue *queue, guint index)
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
               gpointer data,
               guint priority)
{
  pqueue->elements[++(pqueue->size)].data = data;
  pqueue->elements[pqueue->size].priority = priority;

  guint *size_p = g_new(guint, 1);
  *size_p = pqueue->size;
  g_hash_table_insert (pqueue->map, pqueue->elements[pqueue->size].data, size_p);
  swim (pqueue, pqueue->size);
}

gpointer
pqueue_pop_minimum (PQueue *pqueue)
{
  if (pqueue_is_empty (pqueue))
    return NULL;

  gpointer minimum = pqueue->elements[1].data;
  swap (pqueue, 1, pqueue->size);
  pqueue->size--;

  guint *p = g_hash_table_lookup (pqueue->map, minimum);
  g_free (p);
  g_hash_table_remove (pqueue->map, minimum);
  sink (pqueue, 1);

  return minimum;
}

void
pqueue_delete (PQueue *pqueue,
               gpointer data)
{
  guint *pos_p;
  pos_p = (guint *) g_hash_table_lookup (pqueue->map, data);
  guint pos = *pos_p;
  gpointer element = pqueue->elements[pos].data;
  swap (pqueue, pos, pqueue->size);
  pqueue->size--;

  guint *p = g_hash_table_lookup (pqueue->map, element);
  g_free (p);
  g_hash_table_remove (pqueue->map, element);

  if (pos <= pqueue->size)
    sink (pqueue, pos);
}

gboolean
pqueue_has_element (PQueue *pqueue,
                    gpointer data)
{
  return g_hash_table_contains (pqueue->map, data);
}

gboolean
pqueue_is_empty (PQueue *pqueue)
{
  return pqueue->size == 0;
}

void
pqueue_free (PQueue *pqueue)
{
  GHashTableIter iter;
  gpointer key, value;

  g_hash_table_iter_init (&iter, pqueue->map);
  while (g_hash_table_iter_next (&iter, &key, &value))
    g_free (value);

  g_hash_table_destroy (pqueue->map);
  g_slice_free1 ((pqueue->max_size + 1) * sizeof (PQelement), pqueue->elements);
  g_slice_free (PQueue, pqueue);
}

