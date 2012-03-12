/*
 * skeltrack-joint.c
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

#include <string.h>
#include "skeltrack-joint.h"

/**
 * skeltrack_joint_get_type:
 *
 * Returns: The registered #GType for #SkeltrackJoint boxed type
 **/
GType
skeltrack_joint_get_type (void)
{
  static GType type = 0;

  if (G_UNLIKELY (type == 0))
    type = g_boxed_type_register_static ("SkeltrackJoint",
                                         (GBoxedCopyFunc) skeltrack_joint_copy,
                                         (GBoxedFreeFunc) skeltrack_joint_free);
  return type;
}

/**
 * skeltrack_joint_copy:
 * @joint: The #SkeltrackJoint to copy
 *
 * Makes an exact copy of a #SkeltrackJoint object.
 *
 * Returns: (transfer full): A newly created #SkeltrackJoint. Use
 * skeltrack_joint_free() to free it.
 **/
gpointer
skeltrack_joint_copy (SkeltrackJoint *joint)
{
  SkeltrackJoint *new_joint;

  new_joint = NULL;
  joint = g_slice_new0 (SkeltrackJoint);

  memcpy (new_joint, joint, sizeof (SkeltrackJoint));

  return new_joint;
}

/**
 * skeltrack_joint_free:
 * @joint: The #SkeltrackJoint to free
 *
 * Frees a #SkeltrackJoint object.
 **/
void
skeltrack_joint_free (SkeltrackJoint *joint)
{
  g_slice_free (SkeltrackJoint, joint);
}


/**
 * skeltrack_joint_free_list:
 * @list: The #SkeltrackJointList to free
 *
 * Frees a #SkeltrackJointList object.
 **/
void
skeltrack_joint_free_list (SkeltrackJointList list)
{
  gint i;
  for (i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
    {
      g_slice_free (SkeltrackJoint, list[i]);
    }
  g_slice_free1 (SKELTRACK_JOINT_MAX_JOINTS * sizeof (SkeltrackJoint *), list);
}

/**
 * skeltrack_joint_list_get_joint:
 * @list: The #SkeltrackJointList
 * @id: The #SkeltrackJointId of the joint to get
 *
 * Gets a joint from a list of skeleton joints.
 *
 * Returns: (transfer full): The #SkeltrackJoint that corresponds to
 * the given @id.
 **/
SkeltrackJoint *
skeltrack_joint_list_get_joint (SkeltrackJointList list, SkeltrackJointId id)
{
  return list[id];
}
