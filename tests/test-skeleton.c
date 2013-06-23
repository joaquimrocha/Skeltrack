/*
 * test-skeleton.c
 *
 * Skeltrack - A Free Software skeleton tracking library
 *
 * Copyright (C) 2012, Igalia S.L.
 *
 * Authors:
 *   Joaquim Rocha <jrocha@igalia.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 3 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <config.h>
#include <glib.h>
#include <skeltrack.h>

#define WIDTH  640
#define HEIGHT 480

#define NUMBER_OF_FILES 12
#define RESOURCES_FOLDER "./resources/"
static gchar *DEPTH_FILES[NUMBER_OF_FILES] = {
  RESOURCES_FOLDER "depth-data-1028894671",
  RESOURCES_FOLDER "depth-data-1045879925",
  RESOURCES_FOLDER "depth-data-1058893191",
  RESOURCES_FOLDER "depth-data-1070905432",
  RESOURCES_FOLDER "depth-data-1166565565",
  RESOURCES_FOLDER "depth-data-1038901490",
  RESOURCES_FOLDER "depth-data-1051883281",
  RESOURCES_FOLDER "depth-data-1064898470",
  RESOURCES_FOLDER "depth-data-1078881076",
  RESOURCES_FOLDER "depth-data-1234568668",
  RESOURCES_FOLDER "depth-data-1399145206",
  RESOURCES_FOLDER "depth-data-82823944"};

typedef struct
{
  guint16 *data;
  guint width;
  guint height;
} Buffer;

typedef struct
{
  SkeltrackSkeleton *skeleton;
  GMainLoop *main_loop;
  Buffer *buffer;
} Fixture;

static void
fixture_setup_main_loop (Fixture       *f,
                         gconstpointer  test_data)
{
  f->skeleton = skeltrack_skeleton_new ();
  f->main_loop = g_main_loop_new (NULL, FALSE);
}

static void
fixture_teardown_main_loop (Fixture       *f,
                            gconstpointer  test_data)
{
  g_object_unref (f->skeleton);
  g_main_loop_unref (f->main_loop);
}

static void
fixture_setup (Fixture       *f,
               gconstpointer  test_data)
{
  f->skeleton = SKELTRACK_SKELETON (skeltrack_skeleton_new ());
}

static void
fixture_teardown (Fixture       *f,
                  gconstpointer  test_data)
{
  g_object_unref (f->skeleton);
}

static guint16 *
read_file_to_buffer (const gchar *name, gsize count, GError *e)
{
  GError *error = NULL;
  guint16 *depth = NULL;
  GFile *new_file = g_file_new_for_path (name);
  GFileInputStream *input_stream = g_file_read (new_file,
                                                NULL,
                                                &error);
  if (error != NULL)
    {
      g_debug ("ERROR: %s", error->message);
    }
  else
    {
      gsize bread = 0;
      depth = g_slice_alloc (count);
      g_input_stream_read_all ((GInputStream *) input_stream,
                               depth,
                               count,
                               &bread,
                               NULL,
                               &error);

      if (error != NULL)
        {
          g_debug ("ERROR: %s", error->message);
        }
    }
  return depth;
}

static guint16 *
reduce_depth_file (const gchar *name,
                   guint reduce_factor,
                   guint *reduced_width,
                   guint *reduced_height)
{
  guint i, j, r_width, r_height;
  guint16 *depth, *reduced_depth;
  GError *error = NULL;
  gsize count = WIDTH * HEIGHT * sizeof (guint16);

  depth = read_file_to_buffer (name, count, error);

  if (depth == NULL)
    return NULL;

  r_width = (WIDTH - WIDTH % reduce_factor) / reduce_factor;
  r_height = (HEIGHT - HEIGHT % reduce_factor) / reduce_factor;
  reduced_depth = g_slice_alloc (r_width * r_height * sizeof (guint16));

  for (i = 0; i < r_width; i++)
    {
      for (j = 0; j < r_height; j++)
        {
          guint index = j * WIDTH * reduce_factor + i * reduce_factor;
          reduced_depth[j * r_width + i] = depth[index];
        }
    }
  *reduced_width = r_width;
  *reduced_height = r_height;

  g_slice_free1 (count, depth);
  return reduced_depth;
}

static gint
get_number_of_valid_joints (SkeltrackJointList list)
{
  gint i;
  gint count = 0;

  for (i = 0; i < SKELTRACK_JOINT_MAX_JOINTS; i++)
    {
      if (list[i] != NULL)
        count++;
    }
  return count;
}

static void
test_init (Fixture      *f,
           gconstpointer test_data)
{
  g_assert (SKELTRACK_IS_SKELETON (f->skeleton));
}

static void
on_track_joints_with_pending_error (GObject *obj,
                                    GAsyncResult *res,
                                    gpointer data)
{
  SkeltrackJointList list;
  GError *error = NULL;

  list = skeltrack_skeleton_track_joints_finish (SKELTRACK_SKELETON (obj),
                                                 res,
                                                 &error);
  g_assert (error != NULL);
  g_assert_cmpint (error->code, ==, G_IO_ERROR_PENDING);
  g_assert (list == NULL);

  g_error_free (error);
}

static void
on_track_joints (GObject *obj, GAsyncResult *res, gpointer data)
{
  SkeltrackJointList list;
  Fixture *f;
  GError *error = NULL;

  f = (Fixture *) data;

  list = skeltrack_skeleton_track_joints_finish (f->skeleton, res, &error);
  g_assert (error == NULL);
  g_assert (list != NULL);

  g_assert_cmpint (get_number_of_valid_joints (list),
                   ==,
                   7);

  g_main_loop_quit (f->main_loop);

  g_slice_free1 (f->buffer->width * f->buffer->height * sizeof (guint16),
                 f->buffer->data);
  g_slice_free (Buffer, f->buffer);
  skeltrack_joint_list_free (list);
}

static void
test_track_joints_number (Fixture *f,
                          gconstpointer test_data)
{
  gchar *file_name;
  Buffer *buffer;
  GError *error = NULL;
  guint reduction, width, height;
  guint16 *depth;

  file_name = (gchar *) test_data;

  g_object_get (f->skeleton, "dimension-reduction", &reduction, NULL);

  depth = reduce_depth_file (file_name,
                             reduction,
                             &width,
                             &height);
  buffer = g_slice_new (Buffer);
  buffer->data = depth;
  buffer->width = width;
  buffer->height = height;
  f->buffer = buffer;
  skeltrack_skeleton_track_joints (f->skeleton,
                                   depth,
                                   width,
                                   height,
                                   NULL,
                                   on_track_joints,
                                   f);
  g_main_loop_run (f->main_loop);
}

static void
test_track_joints_number_sync (Fixture *f,
                               gconstpointer test_data)
{
  SkeltrackJoint *head;
  gchar *file_name;
  GError *error = NULL;
  SkeltrackJointList list;
  guint reduction, width, height, i;
  guint16 *depth;

  file_name = (gchar *) test_data;
  g_object_get (f->skeleton, "dimension-reduction", &reduction, NULL);

  depth = reduce_depth_file (file_name,
                             reduction,
                             &width,
                             &height);

  list = skeltrack_skeleton_track_joints_sync (f->skeleton,
                                               depth,
                                               width,
                                               height,
                                               NULL,
                                               &error);
  g_assert (list != NULL);
  g_assert (error == NULL);

  g_assert_cmpint (get_number_of_valid_joints (list),
                   ==,
                   7);

  g_slice_free1 (width * height * sizeof (guint16), depth);
  skeltrack_joint_list_free (list);
}

static void
test_pending_operation (Fixture *f,
                        gconstpointer test_data)
{
  Buffer *buffer;
  GError *error = NULL;
  SkeltrackJointList list;
  guint reduction, width, height;
  guint16 *depth;

  g_object_get (f->skeleton, "dimension-reduction", &reduction, NULL);

  depth = reduce_depth_file (DEPTH_FILES[0],
                             reduction,
                             &width,
                             &height);
  buffer = g_slice_new (Buffer);
  buffer->data = depth;
  buffer->width = width;
  buffer->height = height;
  f->buffer = buffer;
  skeltrack_skeleton_track_joints (f->skeleton,
                                   depth,
                                   width,
                                   height,
                                   NULL,
                                   on_track_joints,
                                   f);

  skeltrack_skeleton_track_joints (f->skeleton,
                                   depth,
                                   width,
                                   height,
                                   NULL,
                                   on_track_joints_with_pending_error,
                                   f);

  list = skeltrack_skeleton_track_joints_sync (f->skeleton,
                                               depth,
                                               width,
                                               height,
                                               NULL,
                                               &error);
  g_assert (error != NULL);
  g_assert_cmpint (error->code, ==, G_IO_ERROR_PENDING);
  g_assert (list == NULL);

  g_main_loop_run (f->main_loop);
}

gint
main (gint argc, gchar **argv)
{
  guint i;
  g_test_init (&argc, &argv, NULL);

  g_test_add ("/skeltrack/skeleton/init",
              Fixture,
              NULL,
              fixture_setup,
              test_init,
              fixture_teardown);

  for (i = 0; i < NUMBER_OF_FILES; i++)
    {
      g_test_add ("/skeltrack/skeleton/track_joints_number",
                  Fixture,
                  DEPTH_FILES[i],
                  fixture_setup_main_loop,
                  test_track_joints_number,
                  fixture_teardown_main_loop);

      g_test_add ("/skeltrack/skeleton/track_joints_number_sync",
                  Fixture,
                  DEPTH_FILES[i],
                  fixture_setup,
                  test_track_joints_number_sync,
                  fixture_teardown);
    }

  g_test_add ("/skeltrack/skeleton/pending_operation",
              Fixture,
              NULL,
              fixture_setup_main_loop,
              test_pending_operation,
              fixture_teardown_main_loop);

  g_test_run ();

  return 0;
}
