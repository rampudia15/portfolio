#include "gst_capture.h"


/* The appsink has received a buffer */
static GstFlowReturn	on_new_sample(GstElement *sink)
{
  GstSample	*sample;
  GstBuffer	*buffer;
  GstMapInfo	map;

  /* Retrieve the buffer */
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (sample)
    {
      buffer = gst_sample_get_buffer(sample);
      gst_buffer_map(buffer, &map, GST_MAP_READ);

      jiwy_execute((unsigned char *)map.data);

      // we don't need the appsink sample anymore                                                    
      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

  return GST_FLOW_ERROR;
}


static gboolean		bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *)data;

  switch (GST_MESSAGE_TYPE(msg)) {

  case GST_MESSAGE_EOS:
    g_print("End of stream\n");
    g_main_loop_quit(loop);
    break;

  case GST_MESSAGE_ERROR: {
    gchar  *debug;
    GError *error;

    gst_message_parse_error(msg, &error, &debug);
    g_free(debug);

    g_printerr("Error: %s\n", error->message);
    g_error_free(error);

    g_main_loop_quit(loop);
    break;
  }
  default:
    break;
  }

  return TRUE;
}


GstFlowReturn		create_gst_pipeline(GstCustomData *gstData, char *device)
{
  gstData->loop = g_main_loop_new(NULL, FALSE);

  /* Create gstreamer elements */
  gstData->pipeline = gst_pipeline_new ("object-detector");
  gstData->source = gst_element_factory_make ("v4l2src", "video-source");
  gstData->decode_filter = gst_element_factory_make ("capsfilter", "decode-filter");
  gstData->convert_filter = gst_element_factory_make ("capsfilter", "convert-filter");
  gstData->converter = gst_element_factory_make("videoconvert", "video-converter");
  gstData->decoder = gst_element_factory_make ("jpegdec", "jpeg-decoder");
  gstData->sink = gst_element_factory_make ("appsink", "app_sink");

  if (!gstData->pipeline)
    {
      g_printerr("Pipeline creation failed. Exiting.\n");
      return GST_FLOW_ERROR;
    }
  if (!gstData->source || !gstData->decode_filter || !gstData->convert_filter
      || !gstData->converter || !gstData->decoder || !gstData->sink)
    {
      g_printerr("Gst element creation failed. Exiting.\n");
      return GST_FLOW_ERROR;
    }

  /* Set up the pipeline */

  /* we set the input filename to the source element */
  g_object_set(G_OBJECT(gstData->source), "device", device, NULL);

  /* Caps to decode to jpeg, maybe useless ? But working for now */
  gstData->decode_caps = gst_caps_new_simple("image/jpeg",
					     "width", G_TYPE_INT, D_WIDTH,
					     "height", G_TYPE_INT, D_HEIGHT,
					     "framerate", GST_TYPE_FRACTION, D_FRAMERATE, 1,
					     NULL);
  /* Caps to convert to raw image */
  gstData->convert_caps = gst_caps_new_simple("video/x-raw",
					      "width", G_TYPE_INT, D_WIDTH,
					      "height", G_TYPE_INT, D_HEIGHT,
					      "framerate", GST_TYPE_FRACTION, D_FRAMERATE, 1,
					      "format", G_TYPE_STRING, "RGB",
					      NULL);

  /* Configure appsink */
  g_object_set(gstData->sink, "emit-signals", TRUE, NULL);
  g_signal_connect(gstData->sink, "new-sample", G_CALLBACK(on_new_sample), NULL);

  /* we add a message handler */
  gstData->bus = gst_pipeline_get_bus(GST_PIPELINE(gstData->pipeline));
  gstData->bus_watch_id = gst_bus_add_watch(gstData->bus, bus_call, gstData->loop);
  gst_object_unref(gstData->bus);

  /* Set the caps to the corresponding element */
  g_object_set(G_OBJECT(gstData->decode_filter), "caps", gstData->decode_caps, NULL);
  g_object_set(G_OBJECT(gstData->convert_filter), "caps", gstData->convert_caps, NULL);


  /* we add all elements into the pipeline */
  gst_bin_add_many(GST_BIN(gstData->pipeline),
		   gstData->source, gstData->decode_filter, gstData->decoder,
		   gstData->converter, gstData->convert_filter, gstData->sink, NULL);

  /* we link the elements together */
  if (!gst_element_link_many(gstData->source,
			     gstData->decode_filter, gstData->decoder,
			     gstData->converter, gstData->convert_filter,
			     gstData->sink, NULL))
    {
      g_printerr("Elements could not be linked.\n");
      gst_object_unref(gstData->pipeline);
      gst_caps_unref(gstData->decode_caps);
      gst_caps_unref(gstData->convert_caps);
      return GST_FLOW_ERROR;
    }

  return GST_FLOW_OK;
}


void		run_gst_loop(GstCustomData *gstData, char *device)
{
  /* Set the pipeline to "playing" state*/
  g_print("Now playing: %s\n", device);
  gst_element_set_state(gstData->pipeline, GST_STATE_PLAYING);

  /* Iterate */
  g_print("Running...\n");
  g_main_loop_run(gstData->loop);
}


void		clean_gst_pipeline(GstCustomData *gstData)
{
  g_print("Returned, stopping playback\n");
  gst_element_set_state(gstData->pipeline, GST_STATE_NULL);

  g_print("Deleting pipeline\n");
  gst_object_unref(GST_OBJECT(gstData->pipeline));
  g_source_remove(gstData->bus_watch_id);
  g_main_loop_unref(gstData->loop);
}
