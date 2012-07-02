/*
 * skeltrack-util.h
 *
 * Skeltrack - A Free Software skeleton tracking library
 * Copyright (C) 2012 Igalia S.L.
 *
 * Authors:
 *  Joaquim Rocha <jrocha@igalia.com>
 *  Eduardo Lima Mitev <elima@igalia.com>
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

#include "skeltrack-joint.h"

typedef struct _Label Label;
typedef struct _Node Node;

struct _Label {
  gint index;
  Label *parent;
  GList *nodes;
  Node *bridge_node;
  Node *to_node;
  gint lower_screen_y;
};

struct _Node {
  gint i;
  gint j;
  gint x;
  gint y;
  gint z;
  GList *neighbors;
  GList *linked_nodes;
  Label *label;
};

Node *        get_closest_node_to_joint        (GList *extremas,
                                                SkeltrackJoint *joint,
                                                gint *distance);
