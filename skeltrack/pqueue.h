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
#include "skeltrack-util.h"

struct _PQueue_element {
  Node  *data;
  guint priority;
};

typedef struct _PQueue_element PQelement;

struct _PQueue {
  PQelement *elements;
  guint *map;
  guint size;
  guint max_size;
  guint width;
  guint height;
};

typedef struct _PQueue PQueue;

PQueue *        pqueue_new                      (guint           max_size,
                                                 guint           width,
                                                 guint           height);

void            pqueue_insert                   (PQueue         *pqueue,
                                                 Node           *data,
                                                 guint           priority);

Node *          pqueue_pop_minimum              (PQueue         *pqueue);

void            pqueue_delete                   (PQueue         *pqueue,
                                                 Node           *data);

gboolean        pqueue_has_element              (PQueue         *pqueue,
                                                 Node           *data);

gboolean        pqueue_is_empty                 (PQueue         *pqueue);

void            pqueue_free                     (PQueue         *pqueue);
