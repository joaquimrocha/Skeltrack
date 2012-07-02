/*
 * skeltrack-util.c
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

#include <glib.h>
#include <math.h>

#include "skeltrack-util.h"

static guint
get_distance_from_joint (Node *node, SkeltrackJoint *joint)
{
  guint dx, dy, dz;
  dx = ABS (node->x - joint->x);
  dy = ABS (node->y - joint->y);
  dz = ABS (node->z - joint->z);
  return sqrt (dx * dx + dy * dy + dz * dz);
}

Node *
get_closest_node_to_joint (GList *extremas,
                           SkeltrackJoint *joint,
                           gint *distance)
{
  GList *current_node;
  gint dist = -1;
  Node *closest_node = NULL;

  for (current_node = g_list_first (extremas);
       current_node != NULL;
       current_node = g_list_next (current_node))
    {
      guint current_dist;
      Node *node = (Node *) current_node->data;
      if (node == NULL)
        continue;

      current_dist = get_distance_from_joint (node, joint);
      if (dist == -1 || current_dist < dist)
        {
          closest_node = node;
          dist = current_dist;
        }
    }
  *distance = dist;
  return closest_node;
}
