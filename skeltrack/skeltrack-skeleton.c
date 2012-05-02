/*
 * skeltrack-skeleton.c
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

/**
 * SECTION:skeltrack-skeleton
 * @short_description: Object that tracks the joints in a human skeleton
 *
 * This object tries to detect joints of the human skeleton.
 *
 * To track the joints, first create an instance of #SkeltrackSkeleton using
 * skeltrack_skeleton_new() and then set a buffer from where the joints will
 * be retrieved using the asynchronous function
 * skeltrack_skeleton_track_joints() and get the list of joints using
 * skeltrack_skeleton_track_joints_finish().
 *
 * A common use case is to use this library together with a Kinect device so
 * an easy way to retrieve the needed buffer is to use the GFreenect library.
 *
 * It currently tracks the joints identified by #SkeltrackJointId .
 *
 * Tracking the skeleton joints can be computational heavy so it is advised that
 * the given buffer's dimension is reduced before setting it. To do it,
 * simply choose the reduction factor and loop through the original buffer
 * (using this factor as a step) and set the reduced buffer's values accordingly.
 * The #SkeltrackSkeleton:dimension-reduction property holds this reduction
 * value and should be changed to the reduction factor used (alternatively you
 * can retrieve its default value and use it in the reduction, if it fits your
 * needs).
 *
 * The skeleton tracking uses a few heuristics that proved to work well for
 * tested cases but they can be tweaked by changing the following properties:
 * #SkeltrackSkeleton:graph-distance-threshold ,
 * #SkeltrackSkeleton:graph-minimum-number-nodes ,
 * #SkeltrackSkeleton:hands-minimum-distance ,
 * #SkeltrackSkeleton:shoulders-minimum-distance,
 * #SkeltrackSkeleton:shoulders-maximum-distance and
 * #SkeltrackSkeleton:shoulders-offset .
 **/
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "skeltrack-skeleton.h"

#define SKELTRACK_SKELETON_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), \
                                             SKELTRACK_TYPE_SKELETON, \
                                             SkeltrackSkeletonPrivate))

#define DIMENSION_REDUCTION 16
#define GRAPH_DISTANCE_THRESHOLD 150
#define GRAPH_MINIMUM_NUMBER_OF_NODES 5
#define HANDS_MINIMUM_DISTANCE 600
#define SHOULDERS_MINIMUM_DISTANCE 200
#define SHOULDERS_MAXIMUM_DISTANCE 500
#define SHOULDERS_OFFSET 350

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

/* private data */
struct _SkeltrackSkeletonPrivate
{
  guint16 *buffer;
  guint buffer_width;
  guint buffer_height;

  GAsyncResult *track_joints_result;

  GList *graph;
  GList *labels;
  Node **node_matrix;
  gint  *distances_matrix;
  GList *lowest_component;

  guint16 dimension_reduction;
  guint16 distance_threshold;
  guint16 min_nr_nodes;

  guint16 hands_minimum_distance;
  guint16 shoulders_minimum_distance;
  guint16 shoulders_maximum_distance;
  guint16 shoulders_offset;

  GThread *dispatch_thread;
  GMutex *dispatch_mutex;
  gboolean abort_dispatch_thread;
};

/* @TODO: Expose these to the user */
static const gfloat SCALE_FACTOR = .0021;
static const gint MIN_DISTANCE = -10.0;

/* Currently searches for head and hands */
static const guint NR_EXTREMAS_TO_SEARCH  = 3;

/* properties */
enum
  {
    PROP_0,
    PROP_DIMENSION_REDUCTION,
    PROP_GRAPH_DISTANCE_THRESHOLD,
    PROP_GRAPH_MIN_NR_NODES,
    PROP_HANDS_MINIMUM_DISTANCE,
    PROP_SHOULDERS_MINIMUM_DISTANCE,
    PROP_SHOULDERS_MAXIMUM_DISTANCE,
    PROP_SHOULDERS_OFFSET
  };


static void     skeltrack_skeleton_class_init         (SkeltrackSkeletonClass *class);
static void     skeltrack_skeleton_init               (SkeltrackSkeleton *self);
static void     skeltrack_skeleton_finalize           (GObject *obj);
static void     skeltrack_skeleton_dispose            (GObject *obj);

static void     skeltrack_skeleton_set_property       (GObject *obj,
                                                       guint prop_id,
                                                       const GValue *value,
                                                       GParamSpec *pspec);
static void     skeltrack_skeleton_get_property       (GObject *obj,
                                                       guint prop_id,
                                                       GValue *value,
                                                       GParamSpec *pspec);

G_DEFINE_TYPE (SkeltrackSkeleton, skeltrack_skeleton, G_TYPE_OBJECT)

static void
skeltrack_skeleton_class_init (SkeltrackSkeletonClass *class)
{
  GObjectClass *obj_class;

  obj_class = G_OBJECT_CLASS (class);

  obj_class->dispose = skeltrack_skeleton_dispose;
  obj_class->finalize = skeltrack_skeleton_finalize;
  obj_class->get_property = skeltrack_skeleton_get_property;
  obj_class->set_property = skeltrack_skeleton_set_property;

  /* install properties */

  /**
   * SkeltrackSkeleton:dimension-reduction
   *
   * The value by which the dimension of the buffer was reduced
   * (in case it was).
   **/
  g_object_class_install_property (obj_class,
                         PROP_DIMENSION_REDUCTION,
                         g_param_spec_uint ("dimension-reduction",
                                            "Dimension reduction",
                                            "The dimension reduction value",
                                            1,
                                            1024,
                                            DIMENSION_REDUCTION,
                                            G_PARAM_READWRITE |
                                            G_PARAM_STATIC_STRINGS));

  /**
   * SkeltrackSkeleton:graph-distance-threshold
   *
   * The value (in mm) for the distance threshold between each node and its
   * neighbors. This means that a node in the graph will only be connected
   * to another if they aren't farther apart then this value.
   **/
  g_object_class_install_property (obj_class,
                         PROP_GRAPH_DISTANCE_THRESHOLD,
                         g_param_spec_uint ("graph-distance-threshold",
                                            "Graph's distance threshold",
                                            "The distance threshold between "
                                            "each node.",
                                            1,
                                            G_MAXUINT16,
                                            GRAPH_DISTANCE_THRESHOLD,
                                            G_PARAM_READWRITE |
                                            G_PARAM_STATIC_STRINGS));

  /**
   * SkeltrackSkeleton:graph-minimum-number-nodes
   *
   * The minimum number of nodes each of the graph's components
   * should have (when it is not fully connected).
   **/
  g_object_class_install_property (obj_class,
                         PROP_GRAPH_MIN_NR_NODES,
                         g_param_spec_uint ("graph-minimum-number-nodes",
                                            "Graph's minimum number of nodes",
                                            "The minimum number of nodes "
                                            "of the graph's components ",
                                            1,
                                            G_MAXUINT16,
                                            GRAPH_MINIMUM_NUMBER_OF_NODES,
                                            G_PARAM_READWRITE |
                                            G_PARAM_STATIC_STRINGS));

  /**
   * SkeltrackSkeleton:hands-minimum-distance
   *
   * The minimum distance (in mm) that each hand should be from its
   * respective shoulder.
   **/
  g_object_class_install_property (obj_class,
                         PROP_HANDS_MINIMUM_DISTANCE,
                         g_param_spec_uint ("hands-minimum-distance",
                                            "Hands' minimum distance from the "
                                            "shoulders",
                                            "The minimum distance (in mm) that "
                                            "each hand should be from its "
                                            "respective shoulder.",
                                            300,
                                            G_MAXUINT,
                                            HANDS_MINIMUM_DISTANCE,
                                            G_PARAM_READWRITE |
                                            G_PARAM_STATIC_STRINGS));

  /**
   * SkeltrackSkeleton:shoulders-minimum-distance
   *
   * The minimum distance (in mm) between each of the shoulders and the head.
   **/
  g_object_class_install_property (obj_class,
                         PROP_SHOULDERS_MINIMUM_DISTANCE,
                         g_param_spec_uint ("shoulders-minimum-distance",
                                            "Shoulders' minimum distance",
                                            "The minimum distance (in mm) "
                                            "between each of the shoulders and "
                                            "the head.",
                                            100,
                                            G_MAXUINT16,
                                            SHOULDERS_MINIMUM_DISTANCE,
                                            G_PARAM_READWRITE |
                                            G_PARAM_STATIC_STRINGS));

  /**
   * SkeltrackSkeleton:shoulders-maximum-distance
   *
   * The maximum distance (in mm) between each of the shoulders and the head.
   **/
  g_object_class_install_property (obj_class,
                         PROP_SHOULDERS_MAXIMUM_DISTANCE,
                         g_param_spec_uint ("shoulders-maximum-distance",
                                            "Shoulders' maximum distance",
                                            "The maximum distance (in mm) "
                                            "between each of the shoulders "
                                            "and the head",
                                            150,
                                            G_MAXUINT16,
                                            SHOULDERS_MAXIMUM_DISTANCE,
                                            G_PARAM_READWRITE |
                                            G_PARAM_STATIC_STRINGS));

  /**
   * SkeltrackSkeleton:shoulders-offset
   *
   * The shoulders are searched below the head using this offset (in mm)
   * vertically and half of this offset horizontally. Changing it will
   * affect where the shoulders will be found.
   **/
  g_object_class_install_property (obj_class,
                         PROP_SHOULDERS_OFFSET,
                         g_param_spec_uint ("shoulders-offset",
                                            "Shoulders' offset",
                                            "The shoulders' offest from the "
                                            "head.",
                                            150,
                                            G_MAXUINT16,
                                            SHOULDERS_OFFSET,
                                            G_PARAM_READWRITE |
                                            G_PARAM_STATIC_STRINGS));

  /* add private structure */
  g_type_class_add_private (obj_class, sizeof (SkeltrackSkeletonPrivate));
}

static void
skeltrack_skeleton_init (SkeltrackSkeleton *self)
{
  SkeltrackSkeletonPrivate *priv;

  priv = SKELTRACK_SKELETON_GET_PRIVATE (self);
  self->priv = priv;

  priv->buffer = NULL;
  priv->buffer_width = 0;
  priv->buffer_height = 0;

  priv->graph = NULL;
  priv->labels = NULL;
  priv->lowest_component = NULL;
  priv->node_matrix = NULL;
  priv->distances_matrix = NULL;

  priv->dimension_reduction = DIMENSION_REDUCTION;
  priv->distance_threshold = GRAPH_DISTANCE_THRESHOLD;

  priv->min_nr_nodes = GRAPH_MINIMUM_NUMBER_OF_NODES;

  priv->hands_minimum_distance = HANDS_MINIMUM_DISTANCE;
  priv->shoulders_minimum_distance = SHOULDERS_MINIMUM_DISTANCE;
  priv->shoulders_maximum_distance = SHOULDERS_MAXIMUM_DISTANCE;
  priv->shoulders_offset = SHOULDERS_OFFSET;

  priv->track_joints_result = NULL;

  priv->dispatch_thread = NULL;
  priv->dispatch_mutex = g_mutex_new ();
}

static void
skeltrack_skeleton_dispose (GObject *obj)
{
  SkeltrackSkeleton *self = SKELTRACK_SKELETON (obj);

  /* stop dispatch thread */
  if (self->priv->dispatch_thread != NULL)
    {
      self->priv->abort_dispatch_thread = TRUE;
      g_thread_join (self->priv->dispatch_thread);

      g_mutex_lock (self->priv->dispatch_mutex);

      self->priv->dispatch_thread = NULL;

      g_slice_free1 (self->priv->buffer_width *
                     self->priv->buffer_height * sizeof (gint),
                     self->priv->distances_matrix);
      self->priv->distances_matrix = NULL;

      g_slice_free1 (self->priv->buffer_width *
                     self->priv->buffer_height * sizeof (Node *),
                     self->priv->node_matrix);
      self->priv->node_matrix = NULL;

      g_mutex_unlock (self->priv->dispatch_mutex);
    }

  G_OBJECT_CLASS (skeltrack_skeleton_parent_class)->dispose (obj);
}

static void
skeltrack_skeleton_finalize (GObject *obj)
{
  SkeltrackSkeleton *self = SKELTRACK_SKELETON (obj);

  g_mutex_free (self->priv->dispatch_mutex);

  G_OBJECT_CLASS (skeltrack_skeleton_parent_class)->finalize (obj);
}

static void
skeltrack_skeleton_set_property (GObject      *obj,
                                 guint         prop_id,
                                 const GValue *value,
                                 GParamSpec   *pspec)
{
  SkeltrackSkeleton *self;

  self = SKELTRACK_SKELETON (obj);

  switch (prop_id)
    {
    case PROP_DIMENSION_REDUCTION:
      self->priv->dimension_reduction = g_value_get_uint (value);
      break;

    case PROP_GRAPH_DISTANCE_THRESHOLD:
      self->priv->distance_threshold = g_value_get_uint (value);
      break;

    case PROP_GRAPH_MIN_NR_NODES:
      self->priv->min_nr_nodes = g_value_get_uint (value);
      break;

    case PROP_HANDS_MINIMUM_DISTANCE:
      self->priv->hands_minimum_distance = g_value_get_uint (value);
      break;

    case PROP_SHOULDERS_MINIMUM_DISTANCE:
      self->priv->shoulders_minimum_distance = g_value_get_uint (value);
      break;

    case PROP_SHOULDERS_MAXIMUM_DISTANCE:
      self->priv->shoulders_maximum_distance = g_value_get_uint (value);
      break;

    case PROP_SHOULDERS_OFFSET:
      self->priv->shoulders_offset = g_value_get_uint (value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (obj, prop_id, pspec);
      break;
    }
}

static void
skeltrack_skeleton_get_property (GObject    *obj,
                                 guint       prop_id,
                                 GValue     *value,
                                 GParamSpec *pspec)
{
  SkeltrackSkeleton *self;

  self = SKELTRACK_SKELETON (obj);

  switch (prop_id)
    {
    case PROP_DIMENSION_REDUCTION:
      g_value_set_uint (value, self->priv->dimension_reduction);
      break;

    case PROP_GRAPH_DISTANCE_THRESHOLD:
      g_value_set_uint (value, self->priv->distance_threshold);
      break;

    case PROP_GRAPH_MIN_NR_NODES:
      g_value_set_uint (value, self->priv->min_nr_nodes);
      break;

    case PROP_HANDS_MINIMUM_DISTANCE:
      g_value_set_uint (value, self->priv->hands_minimum_distance);
      break;

    case PROP_SHOULDERS_MINIMUM_DISTANCE:
      g_value_set_uint (value, self->priv->shoulders_minimum_distance);
      break;

    case PROP_SHOULDERS_MAXIMUM_DISTANCE:
      g_value_set_uint (value, self->priv->shoulders_maximum_distance);
      break;

    case PROP_SHOULDERS_OFFSET:
      g_value_set_uint (value, self->priv->shoulders_offset);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (obj, prop_id, pspec);
      break;
    }
}

static void
convert_screen_coords_to_mm (SkeltrackSkeleton *self,
                             guint i,
                             guint j,
                             gint  z,
                             gint *x,
                             gint *y)
{
  guint width = self->priv->buffer_width;
  guint height = self->priv->buffer_height;
  guint dimension_reduction = self->priv->dimension_reduction;

  /* Formula from http://openkinect.org/wiki/Imaging_Information */
  *x = round((i * dimension_reduction - width * dimension_reduction / 2.0) *
             (z + MIN_DISTANCE) * SCALE_FACTOR * (width / height));
  *y = round((j * dimension_reduction - height * dimension_reduction / 2.0) *
             (z + MIN_DISTANCE) * SCALE_FACTOR);
}

static gint
get_distance (Node *a, Node *b)
{
  guint dx, dy, dz;
  dx = ABS (a->x - b->x);
  dy = ABS (a->y - b->y);
  dz = ABS (a->z - b->z);
  return sqrt (dx * dx + dy * dy + dz * dz);
}

static Node *
get_closest_node (GList *node_list, Node *from)
{
  Node *closest = NULL;
  gint distance = -1;
  GList *current_node;

  /* @TODO: Replace this and use closest pair of points
     algorithm and ensure O(n log n) instead of brute-force */

  for (current_node = g_list_first (node_list);
       current_node != NULL;
       current_node = g_list_next (current_node))
    {
      Node *node;
      gint current_distance;
      node = (Node *) current_node->data;
      if (closest == NULL)
        {
          closest = node;
          distance = get_distance (node, from);
          continue;
        }
      current_distance = get_distance (node, from);
      if (current_distance < distance)
        {
          closest = node;
          distance = current_distance;
        }
    }
  return closest;
}

static Label *
label_find (Label *label)
{
  Label *parent;

  g_return_val_if_fail (label != NULL, NULL);

  parent = label->parent;
  if (parent == label)
    return parent;
  else
    return label_find (parent);
}

static void
label_union (Label *a, Label *b)
{
  Label *root_a, *root_b;
  root_a = label_find (a);
  root_b = label_find (b);
  if (root_a->index < root_b->index)
    {
      b->parent = root_a;
    }
  else
    {
      a->parent = root_b;
    }
}

static void
unlink_node (Node *node)
{
  Node *neighbor;
  GList *current_neighbor;

  for (current_neighbor = g_list_first (node->neighbors);
       current_neighbor != NULL;
       current_neighbor = g_list_next (current_neighbor))
    {
      neighbor = (Node *) current_neighbor->data;
      neighbor->neighbors = g_list_remove (neighbor->neighbors, node);
    }

  for (current_neighbor = g_list_first (node->linked_nodes);
       current_neighbor != NULL;
       current_neighbor = g_list_next (current_neighbor))
    {
      neighbor = (Node *) current_neighbor->data;
      neighbor->linked_nodes = g_list_remove (neighbor->linked_nodes, node);
    }

  g_list_free (node->neighbors);
  g_list_free (node->linked_nodes);
  node->neighbors = NULL;
  node->linked_nodes = NULL;
}

static void
free_label (Label *label)
{
  g_list_free (label->nodes);
  label->nodes = NULL;
  g_slice_free (Label, label);
}

static void
clean_labels (GList *labels)
{
  GList *current = g_list_first (labels);
  while (current != NULL)
    {
      Label *label;
      label = (Label *) current->data;
      free_label (label);
      current = g_list_next (current);
    }
}

static void
free_node (Node *node, gboolean unlink_node_first)
{
  if (unlink_node_first)
    {
      unlink_node (node);
    }
  else
    {
      g_list_free (node->neighbors);
      g_list_free (node->linked_nodes);
      node->neighbors = NULL;
      node->linked_nodes = NULL;
    }
  g_slice_free (Node, node);
}

static void
clean_nodes (GList *nodes)
{
  GList *current = g_list_first (nodes);
  while (current != NULL)
    {
      Node *node;
      node = (Node *) current->data;
      free_node (node, FALSE);
      current = g_list_next (current);
    }
}

static gint
join_neighbor (SkeltrackSkeleton *self,
               Node *node,
               Label **neighbor_labels,
               gint index,
               gint i,
               gint j)
{
  Node *neighbor;
  if (i < 0 || i >= self->priv->buffer_width ||
      j < 0 || j >= self->priv->buffer_height)
    {
      return index;
    }

  neighbor = self->priv->node_matrix[self->priv->buffer_width * j + i];
  if (neighbor != NULL)
    {
      gint distance;
      distance = get_distance (neighbor, node);
      if (distance < self->priv->distance_threshold)
        {
          neighbor->neighbors = g_list_append (neighbor->neighbors,
                                               node);
          node->neighbors = g_list_append (node->neighbors,
                                           neighbor);
          neighbor_labels[index] = neighbor->label;
          index++;
        }
    }
  return index;
}

static Label *
get_lowest_index_label (Label **neighbor_labels)
{
  guint index;
  Label *lowest_index_label = NULL;

  lowest_index_label = neighbor_labels[0];
  for (index = 1; index < 4; index++)
    {
      if (neighbor_labels[index] == NULL)
        continue;

      if (lowest_index_label == NULL ||
          lowest_index_label->index < neighbor_labels[index]->index)
        {
          lowest_index_label = neighbor_labels[index];
        }
    }

  return lowest_index_label;
}

static Label *
new_label (gint index)
{
  Label *label = g_slice_new (Label);
  label->index = index;
  label->parent = label;
  label->nodes = NULL;
  label->bridge_node = NULL;
  label->to_node = NULL;
  label->lower_screen_y = -1;

  return label;
}

static void
join_components_to_lowest (GList *nodes,
                           GList *lowest_component,
                           Label *lowest_component_label)
{
  GList *current_node;

  for (current_node = g_list_first (nodes);
       current_node != NULL;
       current_node = g_list_next (current_node))
    {
      Node *node;
      node = (Node *) current_node->data;
      if (node->label == lowest_component_label)
        {
          continue;
        }

      if (node->label->bridge_node == NULL)
        {
          node->label->bridge_node = node;
          node->label->to_node = get_closest_node (lowest_component, node);
        }
      else if (node->label->bridge_node != NULL &&
               node->label->to_node != NULL)
        {
          Node *closest_node;
          gint bridge_distance, distance;
          bridge_distance = get_distance (node->label->bridge_node,
                                          node->label->to_node);
          closest_node = get_closest_node (lowest_component, node);
          distance = get_distance (node,
                                   closest_node);
          if (bridge_distance > distance)
            {
              node->label->bridge_node = node;
              node->label->to_node = closest_node;
            }
        }
    }
}

static GList *
remove_nodes_with_label (GList *nodes,
                         Node **node_matrix,
                         gint width,
                         Label *label)
{
  Node *node;
  GList *link_to_delete, *current_node;

  current_node = g_list_first (nodes);
  while (current_node != NULL)
    {
      node = (Node *) current_node->data;
      if (node->label == label)
        {
          link_to_delete = current_node;
          current_node = g_list_next (current_node);
          nodes = g_list_delete_link (nodes, link_to_delete);
          node_matrix[width * node->j + node->i] = NULL;
          free_node (node, TRUE);
          continue;
        }
      current_node = g_list_next (current_node);
    }
  return nodes;
}

GList *
make_graph (SkeltrackSkeleton *self, GList **label_list)
{
  SkeltrackSkeletonPrivate *priv;
  gint i, j, n;
  Node *node;
  GList *nodes = NULL;
  GList *labels = NULL;
  GList *current_label;
  Label *lowest_component_label = NULL;
  gint index = 0;
  gint next_label = -1;
  guint16 value;
  guint16 *buffer;
  gint width, height;

  buffer = self->priv->buffer;
  width = self->priv->buffer_width;
  height = self->priv->buffer_height;

  priv = self->priv;


  if (priv->node_matrix == NULL)
    {
      priv->node_matrix = g_slice_alloc0 (width * height * sizeof (Node *));
    }
  else
    {
      memset (self->priv->node_matrix,
              0,
              width * height * sizeof (Node *));
    }

  for (i = 0; i < width; i++)
    {
      for (j = 0; j < height; j++)
        {
          gint south, north, west;
          Label *lowest_index_label = NULL;
          Label *neighbor_labels[4] = {NULL, NULL, NULL, NULL};

          value = buffer[j * width + i];
          if (value == 0)
            continue;

          node = g_slice_new0 (Node);
          node->i = i;
          node->j = j;
          node->z = value;
          convert_screen_coords_to_mm (self,
                                       i, j,
                                       node->z,
                                       &(node->x),
                                       &(node->y));
          node->neighbors = NULL;
          node->linked_nodes = NULL;

          index = 0;

          south = j + 1;
          north = j - 1;
          west = i - 1;

          /* West */
          index = join_neighbor (self,
                                 node,
                                 neighbor_labels,
                                 index,
                                 west, j);
          /* South West*/
          index = join_neighbor (self,
                                 node,
                                 neighbor_labels,
                                 index,
                                 west, south);
          /* North */
          index = join_neighbor (self,
                                 node,
                                 neighbor_labels,
                                 index,
                                 i, north);

          /* North West */
          index = join_neighbor (self,
                                 node,
                                 neighbor_labels,
                                 index,
                                 west, north);

          lowest_index_label = get_lowest_index_label (neighbor_labels);

          /* No neighbors */
          if (lowest_index_label == NULL)
            {
              Label *label;
              next_label++;
              label = new_label (next_label);
              labels = g_list_append (labels, label);
              lowest_index_label = label;
            }
          else
            {
              for (index = 0; index < 4; index++)
                {
                  if (neighbor_labels[index] != NULL)
                    {
                      label_union (neighbor_labels[index], lowest_index_label);
                    }
                }
            }

          node->label = lowest_index_label;
          nodes = g_list_append(nodes, node);
          priv->node_matrix[width * node->j + node->i] = node;
        }
    }

  for (n = 0; n < g_list_length (nodes); n++)
    {
      Node *node = (Node *) g_list_nth_data (nodes, n);
      node->label = label_find (node->label);
      node->label->nodes = g_list_append (node->label->nodes,
                                          node);

      /* Assign lower node so we can extract the
         lower graph's component */
      if (g_list_length (node->label->nodes) >= priv->min_nr_nodes &&
          (node->label->lower_screen_y == -1 ||
           node->j > node->label->lower_screen_y))
        {
          node->label->lower_screen_y = node->j;
        }
    }

  current_label = g_list_first (labels);
  while (current_label != NULL)
    {
      Label *label;
      label = (Label *) current_label->data;

      /* Remove label if number of nodes is less than
         the minimum required */
      if (g_list_length (label->nodes) < priv->min_nr_nodes)
        {
          nodes = remove_nodes_with_label (nodes,
                                           priv->node_matrix,
                                           priv->buffer_width,
                                           label);

          GList *link = current_label;
          current_label = g_list_next (current_label);
          labels = g_list_delete_link (labels, link);
          free_label (label);
          continue;
        }

      /* Get the lowest component label */
      if (lowest_component_label == NULL ||
          lowest_component_label->lower_screen_y < label->lower_screen_y ||
          (lowest_component_label->lower_screen_y == label->lower_screen_y &&
           g_list_length (label->nodes) >
           g_list_length (lowest_component_label->nodes)))
        {
          lowest_component_label = label;
        }

      current_label = g_list_next (current_label);
    }

  if (labels)
    {
      priv->lowest_component = lowest_component_label->nodes;

      join_components_to_lowest (nodes,
                                 priv->lowest_component,
                                 lowest_component_label);

      for (current_label = g_list_first (labels);
           current_label != NULL;
           current_label = g_list_next (current_label))
        {
          Label *label;
          label = (Label *) current_label->data;
          if (label == lowest_component_label)
            continue;

          /* @TODO: Remove labels that are too far away
             (to avoid e.g. including objects away from the user) */

          label->bridge_node->neighbors =
            g_list_append (label->bridge_node->neighbors, label->to_node);
          label->to_node->neighbors = g_list_append (label->to_node->neighbors,
                                                     label->bridge_node);
        }
    }

  *label_list = labels;

  return nodes;
}

Node *
get_centroid (SkeltrackSkeleton *self)
{
  gint avg_x = 0;
  gint avg_y = 0;
  gint avg_z = 0;
  gint length;
  GList *node_list;
  Node *cent = NULL;
  Node *centroid = NULL;

  if (self->priv->lowest_component == NULL)
    return NULL;

  for (node_list = g_list_first (self->priv->lowest_component);
       node_list != NULL;
       node_list = g_list_next (node_list))
    {
      Node *node;
      node = (Node *) node_list->data;
      avg_x += node->x;
      avg_y += node->y;
      avg_z += node->z;
    }

  length = g_list_length (self->priv->lowest_component);
  cent = g_slice_new0 (Node);
  cent->x = avg_x / length;
  cent->y = avg_y / length;
  cent->z = avg_z / length;
  cent->linked_nodes = NULL;

  centroid = get_closest_node (self->priv->graph, cent);

  g_slice_free (Node, cent);

  return centroid;
}

static Node *
get_lowest (SkeltrackSkeleton *self, Node *centroid)
{
  Node *lowest = NULL;
  /* @TODO: Use the node_matrix instead of the lowest
     component to look for the lowest node as it's faster. */
  if (self->priv->lowest_component != NULL)
    {
      GList *node_list;
      for (node_list = g_list_first (self->priv->lowest_component);
           node_list != NULL;
           node_list = g_list_next (node_list))
        {
          Node *node;
          node = (Node *) node_list->data;
          if (node->i != centroid->i)
            continue;
          if (lowest == NULL ||
              lowest->j < node->j)
            {
              lowest = node;
            }
        }
    }
  return lowest;
}

static Node *
get_longer_distance (SkeltrackSkeleton *self, gint *distances)
{
  GList *current;
  Node *farthest_node;

  current = g_list_first (self->priv->graph);
  farthest_node = (Node *) current->data;
  current = g_list_next (current);

  while (current != NULL)
    {
      Node *node;
      node = (Node *) current->data;
      if (node != NULL &&
          (distances[farthest_node->j *
                     self->priv->buffer_width +
                     farthest_node->i] != -1 &&
           distances[farthest_node->j *
                     self->priv->buffer_width +
                     farthest_node->i] < distances[node->j *
                                                   self->priv->buffer_width +
                                                   node->i]))
        {
          farthest_node = node;
        }
      current = g_list_next (current);
    }
  return farthest_node;
}

static gboolean
dijkstra_to (GList *nodes, Node *source, Node *target,
             gint width, gint height,
             gint *distances, Node **previous)
{
  gint nr;
  GList *unvisited_nodes, *current;

  for (current = g_list_first (nodes);
       previous != NULL && current != NULL;
       current = g_list_next (current))
    {
      Node *node;
      node = (Node *) current->data;
      previous[node->j * width + node->i] = NULL;
    }
  distances[source->j * width + source->i] = 0;

  unvisited_nodes = g_list_copy (nodes);
  nr = 0;
  while (unvisited_nodes != NULL)
    {
      Node *node;
      GList *current_neighbor, *shorter_dist_node, *cur_node;

      shorter_dist_node = g_list_first (unvisited_nodes);
      cur_node = g_list_next (shorter_dist_node);
      while (cur_node != NULL)
        {
          Node *value, *shorter_dist;
          value = (Node *) cur_node->data;
          shorter_dist = (Node *) shorter_dist_node->data;
          if (distances[shorter_dist->j * width + shorter_dist->i] == -1 ||
              (distances[value->j * width + value->i] != -1 &&
               distances[value->j * width +
                         value->i] < distances[shorter_dist->j * width +
                                               shorter_dist->i]))
            {
              shorter_dist_node = cur_node;
            }
          cur_node = g_list_next (cur_node);
        }

      node = (Node *) shorter_dist_node->data;
      if (distances[node->j * width + node->i] == -1)
        {
          break;
        }

      current_neighbor = g_list_first (node->neighbors);
      while (current_neighbor)
        {
          gint dist;
          Node *neighbor;

          neighbor = (Node *) current_neighbor->data;
          dist = get_distance (node, neighbor) +
            distances[node->j * width + node->i];

          if (distances[neighbor->j * width + neighbor->i] == -1 ||
              dist < distances[neighbor->j * width + neighbor->i])
            {
              distances[neighbor->j * width + neighbor->i] = dist;
              if (previous != NULL)
                {
                  previous[neighbor->j * width + neighbor->i] = node;
                }
              nr++;
            }
          if (target != NULL && neighbor == target)
            {
              g_list_free (unvisited_nodes);
              return TRUE;
            }

          current_neighbor = g_list_next (current_neighbor);
        }
      unvisited_nodes = g_list_delete_link (unvisited_nodes, shorter_dist_node);
    }
  g_list_free (unvisited_nodes);
  return FALSE;
}

static GList *
get_extremas (SkeltrackSkeleton *self, Node *centroid)
{
  SkeltrackSkeletonPrivate *priv;
  gint i, nr_nodes, matrix_size;
  Node *lowest, *source, *node;
  GList *extremas = NULL;

  priv = self->priv;
  lowest = get_lowest (self, centroid);
  source = lowest;

  matrix_size = priv->buffer_width * priv->buffer_height;
  if (priv->distances_matrix == NULL)
    {
      priv->distances_matrix = g_slice_alloc0 (matrix_size * sizeof (gint));
    }

  for (i = 0; i < matrix_size; i++)
    {
      priv->distances_matrix[i] = -1;
    }

  for (nr_nodes = NR_EXTREMAS_TO_SEARCH;
       source != NULL && nr_nodes > 0;
       nr_nodes--)
    {
      dijkstra_to (priv->graph,
                   source,
                   NULL,
                   priv->buffer_width,
                   priv->buffer_height,
                   priv->distances_matrix,
                   NULL);

      node = get_longer_distance (self, priv->distances_matrix);
      if (node == NULL)
        continue;

      if (node != source)
        {
          priv->distances_matrix[node->j * priv->buffer_width + node->i] = 0;
          source->linked_nodes = g_list_append (source->linked_nodes, node);
          node->linked_nodes = g_list_append (node->linked_nodes, source);
          source = node;
          extremas = g_list_append (extremas, node);
        }
    }

  return extremas;
}

static gboolean
get_head_and_shoulders (GList *nodes,
                        guint16 shoulders_minimum_distance,
                        guint16 shoulders_maximum_distance,
                        guint16 shoulders_offset,
                        GList *extremas,
                        Node *centroid,
                        Node **head,
                        Node **left_shoulder,
                        Node **right_shoulder)
{
  Node *node, *shoulder_point;
  Node *right_shoulder_closest_point, *left_shoulder_closest_point;
  gint right_shoulder_dist, left_shoulder_dist, shoulders_distance;
  GList *current_extrema;

  for (current_extrema = g_list_first (extremas);
       current_extrema != NULL;
       current_extrema = g_list_next (current_extrema))
    {
      node = (Node *) current_extrema->data;

      if (node->j > centroid->j)
        continue;

      shoulder_point = g_slice_new0 (Node);
      shoulder_point->x = node->x - shoulders_offset / 2;
      shoulder_point->y = node->y + shoulders_offset;
      shoulder_point->z = node->z;

      right_shoulder_closest_point = get_closest_node (nodes, shoulder_point);
      if (right_shoulder_closest_point->i > centroid->i)
        {
          g_slice_free (Node, shoulder_point);
          continue;
        }

      shoulder_point->x = node->x + shoulders_offset / 2;

      left_shoulder_closest_point = get_closest_node (nodes, shoulder_point);
      if (left_shoulder_closest_point->i < centroid->i)
        {
          g_slice_free (Node, shoulder_point);
          continue;
        }

      right_shoulder_dist = get_distance (node, right_shoulder_closest_point);
      left_shoulder_dist = get_distance (node, left_shoulder_closest_point);
      shoulders_distance = get_distance (left_shoulder_closest_point,
                                         right_shoulder_closest_point);

      if (right_shoulder_closest_point->i < node->i &&
          right_shoulder_dist > shoulders_minimum_distance &&
          right_shoulder_dist < shoulders_maximum_distance &&
          left_shoulder_closest_point->i > node->i &&
          left_shoulder_dist > shoulders_minimum_distance &&
          left_shoulder_dist < shoulders_maximum_distance &&
          ABS (right_shoulder_closest_point->y -
               left_shoulder_closest_point->y) < shoulders_maximum_distance &&
          shoulders_distance <= shoulders_maximum_distance)
        {
          *head = node;
          *right_shoulder = right_shoulder_closest_point;
          *left_shoulder = left_shoulder_closest_point;
          return TRUE;
        }
    }
  return FALSE;
}

static gint *
create_new_dist_matrix (gint matrix_size)
{
  guint i;
  gint *distances;

  distances = g_slice_alloc0 (matrix_size * sizeof (gint));
  for (i = 0; i < matrix_size; i++)
    {
      distances[i] = -1;
    }

  return distances;
}

static SkeltrackJoint *
node_to_joint (Node *node, SkeltrackJointId id, gint dimension_reduction)
{
  SkeltrackJoint *joint;

  if (node == NULL)
    return NULL;

  joint = g_slice_new0 (SkeltrackJoint);
  joint->id = id;
  joint->x = node->x;
  joint->y = node->y;
  joint->z = node->z;
  joint->screen_x = node->i * dimension_reduction;
  joint->screen_y = node->j * dimension_reduction;

  return joint;
}

static void
set_joint_from_node (SkeltrackJointList *joints,
                     Node *node,
                     SkeltrackJointId id,
                     gint dimension_reduction)
{
  (*joints)[id] = node_to_joint (node, id, dimension_reduction);
}

static void
identify_arm_extrema (gint *distances,
                      Node **previous_nodes,
                      gint width,
                      gint hand_distance,
                      Node *extrema,
                      Node **elbow_extrema,
                      Node **hand_extrema)
{
  gint total_dist;

  if (extrema == NULL)
    return;

  total_dist = distances[width * extrema->j + extrema->i];
  if (total_dist < hand_distance)
    {
      *elbow_extrema = extrema;
      *hand_extrema = NULL;
    }
  else
    {
      Node *previous;
      gint elbow_dist;

      previous = previous_nodes[extrema->j * width + extrema->i];
      elbow_dist = total_dist / 2;
      while (previous &&
             distances[previous->j * width + previous->i] > elbow_dist)
        {
          previous = previous_nodes[previous->j * width + previous->i];
        }
      *elbow_extrema = previous;
      *hand_extrema = extrema;
    }
}

static void
assign_extremas_to_left_and_right (Node *extrema_a,
                                   Node *extrema_b,
                                   gint distance_left_a,
                                   gint distance_left_b,
                                   gint distance_right_a,
                                   gint distance_right_b,
                                   Node **left_extrema,
                                   Node **right_extrema)
{
  gint index;
  gint current_distance;
  gint shortest_distance;
  gint shortest_dist_index = 0;
  gint distances[4] = {distance_left_a, distance_left_b,
                       distance_right_a, distance_right_b};

  for (index = 1; index < 4; index++)
    {
      current_distance = distances[index];
      shortest_distance = distances[shortest_dist_index];
      if (current_distance < shortest_distance)
        shortest_dist_index = index;
    }

  if (shortest_dist_index == 0 ||
      shortest_dist_index == 3)
    {
      *left_extrema = extrema_a;
      *right_extrema = extrema_b;
    }
  else if (shortest_dist_index == 1||
           shortest_dist_index == 2)
    {
      *left_extrema = extrema_b;
      *right_extrema = extrema_a;
    }
}

static void
set_left_and_right_from_extremas (SkeltrackSkeleton *self,
                                  GList *extremas,
                                  Node *head,
                                  Node *left_shoulder,
                                  Node *right_shoulder,
                                  SkeltrackJointList *joints)
{
  gint *dist_left_a = NULL;
  gint *dist_left_b = NULL;
  gint *dist_right_a = NULL;
  gint *dist_right_b = NULL;
  gint total_dist_left_a = -1;
  gint total_dist_right_a = -1;
  gint total_dist_left_b = -1;
  gint total_dist_right_b = -1;
  gint *distances_right, *distances_left;
  Node *right, *left, *elbow_extrema, *hand_extrema;
  Node **previous_right, **previous_left;
  Node **previous_left_a = NULL;
  Node **previous_left_b = NULL;
  Node **previous_right_a = NULL;
  Node **previous_right_b = NULL;
  Node *ext_a = NULL;
  Node *ext_b = NULL;
  GList *current_extrema;
  gint width, height, matrix_size;

  for (current_extrema = g_list_first (extremas);
       current_extrema != NULL;
       current_extrema = g_list_next (current_extrema))
    {
      Node *node;
      node = (Node *) current_extrema->data;
      if (node != head)
        {
          if (ext_a == NULL)
            ext_a = node;
          else
            ext_b = node;
        }
    }

  if (head == NULL)
    return;

  width = self->priv->buffer_width;
  height = self->priv->buffer_height;
  matrix_size = width * height;

  previous_left_a = g_slice_alloc0 (matrix_size * sizeof (Node *));
  previous_left_b = g_slice_alloc0 (matrix_size * sizeof (Node *));
  previous_right_a = g_slice_alloc0 (matrix_size * sizeof (Node *));
  previous_right_b = g_slice_alloc0 (matrix_size * sizeof (Node *));

  dist_left_a = create_new_dist_matrix(matrix_size);
  dijkstra_to (self->priv->graph,
               left_shoulder,
               ext_a,
               width,
               height,
               dist_left_a,
               previous_left_a);

  dist_left_b = create_new_dist_matrix(matrix_size);
  dijkstra_to (self->priv->graph,
               left_shoulder,
               ext_b,
               width,
               height,
               dist_left_b,
               previous_left_b);

  dist_right_a = create_new_dist_matrix(matrix_size);
  dijkstra_to (self->priv->graph,
               right_shoulder,
               ext_a,
               width,
               height,
               dist_right_a, previous_right_a);

  dist_right_b = create_new_dist_matrix(matrix_size);
  dijkstra_to (self->priv->graph,
               right_shoulder,
               ext_b,
               width,
               height,
               dist_right_b,
               previous_right_b);

  total_dist_left_a = dist_left_a[ext_a->j * width + ext_a->i];
  total_dist_right_a = dist_right_a[ext_a->j * width + ext_a->i];
  total_dist_left_b = dist_left_b[ext_b->j * width + ext_b->i];
  total_dist_right_b = dist_right_b[ext_b->j * width + ext_b->i];

  left = NULL;
  right = NULL;
  assign_extremas_to_left_and_right (ext_a,
                                     ext_b,
                                     total_dist_left_a,
                                     total_dist_left_b,
                                     total_dist_right_a,
                                     total_dist_right_b,
                                     &left,
                                     &right);
  if (left == ext_a)
    {
      distances_left = dist_left_a;
      previous_left = previous_left_a;
      distances_right = dist_right_b;
      previous_right = previous_right_b;
    }
  else
    {
      distances_left = dist_left_b;
      previous_left = previous_left_b;
      distances_right = dist_right_a;
      previous_right = previous_right_a;
    }

  elbow_extrema = NULL;
  hand_extrema = NULL;
  identify_arm_extrema (distances_left,
                        previous_left,
                        width,
                        self->priv->hands_minimum_distance,
                        left,
                        &elbow_extrema,
                        &hand_extrema);
  set_joint_from_node (joints,
                       elbow_extrema,
                       SKELTRACK_JOINT_ID_LEFT_ELBOW,
                       self->priv->dimension_reduction);
  set_joint_from_node (joints,
                       hand_extrema,
                       SKELTRACK_JOINT_ID_LEFT_HAND,
                       self->priv->dimension_reduction);

  elbow_extrema = NULL;
  hand_extrema = NULL;
  identify_arm_extrema (distances_right,
                        previous_right,
                        width,
                        self->priv->hands_minimum_distance,
                        right,
                        &elbow_extrema,
                        &hand_extrema);
  set_joint_from_node (joints,
                       elbow_extrema,
                       SKELTRACK_JOINT_ID_RIGHT_ELBOW,
                       self->priv->dimension_reduction);
  set_joint_from_node (joints,
                       hand_extrema,
                       SKELTRACK_JOINT_ID_RIGHT_HAND,
                       self->priv->dimension_reduction);

  g_slice_free1 (matrix_size * sizeof (Node *), previous_left_a);
  g_slice_free1 (matrix_size * sizeof (Node *), previous_left_b);
  g_slice_free1 (matrix_size * sizeof (Node *), previous_right_a);
  g_slice_free1 (matrix_size * sizeof (Node *), previous_right_b);

  g_slice_free1 (matrix_size * sizeof (gint), dist_left_a);
  g_slice_free1 (matrix_size * sizeof (gint), dist_left_b);
  g_slice_free1 (matrix_size * sizeof (gint), dist_right_a);
  g_slice_free1 (matrix_size * sizeof (gint), dist_right_b);
}

static SkeltrackJoint **
track_joints (SkeltrackSkeleton *self)
{
  Node * centroid;
  Node *head = NULL;
  Node *right_shoulder = NULL;
  Node *left_shoulder = NULL;
  GList *extremas;
  SkeltrackJoint **joints;

  self->priv->graph = make_graph (self, &self->priv->labels);
  centroid = get_centroid (self);
  extremas = get_extremas (self, centroid);

  joints = g_slice_alloc0 (SKELTRACK_JOINT_MAX_JOINTS *
                           sizeof (SkeltrackJoint *));

  if (g_list_length (extremas) > 2 &&
      get_head_and_shoulders (self->priv->graph,
                              self->priv->shoulders_minimum_distance,
                              self->priv->shoulders_maximum_distance,
                              self->priv->shoulders_offset,
                              extremas,
                              centroid,
                              &head,
                              &left_shoulder,
                              &right_shoulder))
    {

      set_joint_from_node (&joints,
                           head,
                           SKELTRACK_JOINT_ID_HEAD,
                           self->priv->dimension_reduction);
      set_joint_from_node (&joints,
                           left_shoulder,
                           SKELTRACK_JOINT_ID_LEFT_SHOULDER,
                           self->priv->dimension_reduction);
      set_joint_from_node (&joints,
                           right_shoulder,
                           SKELTRACK_JOINT_ID_RIGHT_SHOULDER,
                           self->priv->dimension_reduction);

      set_left_and_right_from_extremas (self,
                                        extremas,
                                        head,
                                        left_shoulder,
                                        right_shoulder,
                                        &joints);
    }

  self->priv->buffer = NULL;

  self->priv->lowest_component = NULL;

  clean_nodes (self->priv->graph);
  g_list_free (self->priv->graph);
  self->priv->graph = NULL;

  clean_labels (self->priv->labels);
  g_list_free (self->priv->labels);
  self->priv->labels = NULL;

  return (SkeltrackJointList) joints;
}

static gpointer
dispatch_thread_func (gpointer _data)
{
  SkeltrackSkeleton *self = SKELTRACK_SKELETON (_data);
  gboolean abort = FALSE;

  while (! abort)
    {
      if (self->priv->track_joints_result != NULL)
        {
          SkeltrackJointList joints = track_joints (self);

          g_mutex_lock (self->priv->dispatch_mutex);

          GSimpleAsyncResult *res =
            (GSimpleAsyncResult *) self->priv->track_joints_result;
          g_simple_async_result_set_op_res_gpointer (res, joints, NULL);

          g_simple_async_result_complete_in_idle (res);
          g_object_unref (self->priv->track_joints_result);
          self->priv->track_joints_result = NULL;

          g_mutex_unlock (self->priv->dispatch_mutex);
        }

      if (self->priv->track_joints_result == NULL ||
          self->priv->abort_dispatch_thread)
        {
          abort = TRUE;
        }
      else
        {
          g_usleep (1);
        }
    }

  g_mutex_lock (self->priv->dispatch_mutex);
  self->priv->dispatch_thread = NULL;
  g_mutex_unlock (self->priv->dispatch_mutex);

  return NULL;
}

static gboolean
launch_dispatch_thread (SkeltrackSkeleton *self, GError **error)
{
  self->priv->abort_dispatch_thread = FALSE;

  self->priv->dispatch_thread = g_thread_create (dispatch_thread_func,
                                                 self,
                                                 TRUE,
                                                 error);
  return self->priv->dispatch_thread != NULL;
}

static void
clean_tracking_resources (SkeltrackSkeleton *self)
{
  g_slice_free1 (self->priv->buffer_width *
                 self->priv->buffer_height * sizeof (gint),
                 self->priv->distances_matrix);
  self->priv->distances_matrix = NULL;

  g_slice_free1 (self->priv->buffer_width *
                 self->priv->buffer_height * sizeof (Node *),
                 self->priv->node_matrix);
  self->priv->node_matrix = NULL;
}

/* public methods */

/**
 * skeltrack_skeleton_new:
 *
 * Constructs and returns a new #SkeltrackSkeleton instance.
 *
 * Returns: (transfer full): The newly created #SkeltrackSkeleton.
 */
GObject *
skeltrack_skeleton_new (void)
{
  return g_object_new (SKELTRACK_TYPE_SKELETON, NULL);
}

/**
 * skeltrack_skeleton_track_joints:
 * @self: The #SkeltrackSkeleton
 * @buffer: The buffer containing the depth information, from which
 * all the information will be retrieved.
 * @width: The width of the @buffer
 * @height: The height of the @buffer
 * @cancellable: (allow-none): A cancellable object, or %NULL (currently
 *  unused)
 * @callback: (scope async): The function to call when the it is finished
 * tracking the joints
 * @user_data: (allow-none): An arbitrary user data to pass in @callback,
 * or %NULL
 *
 * Tracks the skeleton's joints.
 *
 * It uses the depth information contained in the given @buffer and tries to
 * track the skeleton joints. The @buffer's depth values should be in mm.
 * Use skeltrack_skeleton_track_joints_finish() to get a list of the joints
 * found.
 *
 * If this method is called while a previous attempt of tracking the joints
 * is still running, a %G_IO_ERROR_PENDING error occurs.
 **/
void
skeltrack_skeleton_track_joints (SkeltrackSkeleton   *self,
                                 guint16             *buffer,
                                 guint                width,
                                 guint                height,
                                 GCancellable        *cancellable,
                                 GAsyncReadyCallback  callback,
                                 gpointer             user_data)
{
  GSimpleAsyncResult *result = NULL;

  g_return_if_fail (SKELTRACK_IS_SKELETON (self) &&
                    callback != NULL &&
                    buffer != NULL);

  result = g_simple_async_result_new (G_OBJECT (self),
                                      callback,
                                      user_data,
                                      skeltrack_skeleton_track_joints);

  if (self->priv->track_joints_result != NULL)
    {
      g_simple_async_result_set_error (result,
                                       G_IO_ERROR,
                                       G_IO_ERROR_PENDING,
                                       "Currently tracking joints");
      g_simple_async_result_complete_in_idle (result);
      g_object_unref (result);
      return;
    }

  g_mutex_lock (self->priv->dispatch_mutex);

  self->priv->track_joints_result = (GAsyncResult *) result;

  /* @TODO: Set the cancellable */

  self->priv->buffer = buffer;

  if (self->priv->buffer_width != width ||
      self->priv->buffer_height != height)
    {
      clean_tracking_resources (self);

      self->priv->buffer_width = width;
      self->priv->buffer_height = height;
    }

  g_mutex_unlock (self->priv->dispatch_mutex);

  if (self->priv->dispatch_thread == NULL)
    launch_dispatch_thread (self, NULL);
}

/**
 * skeltrack_skeleton_track_joints_finish:
 * @self: The #SkeltrackSkeleton
 * @result: The #GAsyncResult provided in the callback
 * @error: (allow-none): A pointer to a #GError, or %NULL
 *
 * Gets the list of joints that were retrieved by a
 * skeltrack_skeleton_track_joints() operation.
 *
 * Use skeltrack_joint_list_get_joint() with #SkeltrackJointId
 * to get the respective joints from the list.
 * Joints that could not be found will appear as %NULL in the list.
 *
 * The list should be freed using skeltrack_joint_list_free().
 *
 * Returns: (transfer full): The #SkeltrackJointList with the joints found.
 */
SkeltrackJointList
skeltrack_skeleton_track_joints_finish (SkeltrackSkeleton *self,
                                        GAsyncResult      *result,
                                        GError           **error)
{
  GSimpleAsyncResult *res;

  g_return_val_if_fail (G_IS_SIMPLE_ASYNC_RESULT (result), NULL);

  res = G_SIMPLE_ASYNC_RESULT (result);

  if (! g_simple_async_result_propagate_error (res, error))
    {
      SkeltrackJointList joints = NULL;
      joints = g_simple_async_result_get_op_res_gpointer (res);
      return joints;
    }
  else
    return NULL;
}

/**
 * skeltrack_skeleton_track_joints_sync:
 * @self: The #SkeltrackSkeleton
 * @buffer: The buffer containing the depth information, from which
 * all the information will be retrieved.
 * @width: The width of the @buffer
 * @height: The height of the @buffer
 * @cancellable: (allow-none): A cancellable object, or %NULL (currently
 *  unused)
 * @error: (allow-none): A pointer to a #GError, or %NULL
 *
 * Tracks the skeleton's joints synchronously.
 *
 * Does the same as skeltrack_skeleton_track_joints() but synchronously
 * and returns the list of joints found.
 * Ideal for off-line skeleton tracking.
 *
 * If this method is called while a previous attempt of asynchronously
 * tracking the joints is still running, a %G_IO_ERROR_PENDING error occurs.
 *
 * The joints list should be freed using skeltrack_joint_list_free().
 *
 * Returns: (transfer full): The #SkeltrackJointList with the joints found.
 **/
SkeltrackJointList
skeltrack_skeleton_track_joints_sync (SkeltrackSkeleton   *self,
                                      guint16             *buffer,
                                      guint                width,
                                      guint                height,
                                      GCancellable        *cancellable,
                                      GError             **error)
{
  g_return_val_if_fail (SKELTRACK_IS_SKELETON (self), NULL);

  if (self->priv->track_joints_result != NULL && error != NULL)
    {
      *error = g_error_new (G_IO_ERROR,
                            G_IO_ERROR_PENDING,
                            "Currently tracking joints");
      return NULL;
    }

  self->priv->buffer = buffer;

  if (self->priv->buffer_width != width ||
      self->priv->buffer_height != height)
    {
      clean_tracking_resources (self);

      self->priv->buffer_width = width;
      self->priv->buffer_height = height;
    }

  return track_joints (self);
}
