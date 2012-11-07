/*
 * pqueue.h
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

#include <glib.h>

struct _PQueue_element {
  gpointer data;
  guint priority;
};

typedef struct _PQueue_element PQelement;

struct _PQueue {
  PQelement *elements;
  GHashTable *map;
  guint size;
  guint max_size;
};

typedef struct _PQueue PQueue;

PQueue *        pqueue_new                      (guint           max_size);

void            pqueue_insert                   (PQueue         *pqueue,
                                                 gpointer        data,
                                                 guint           priority);

gpointer        pqueue_pop_minimum              (PQueue         *pqueue);

void            pqueue_delete                   (PQueue         *pqueue,
                                                 gpointer        data);

gboolean        pqueue_has_element              (PQueue         *pqueue,
                                                 gpointer        data);

gboolean        pqueue_is_empty                 (PQueue         *pqueue);

void            pqueue_free                     (PQueue         *pqueue);
