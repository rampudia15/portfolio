#ifndef GST_CAPTURE_H_
#define GST_CAPTURE_H_

#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>


#define D_WIDTH		320
#define D_HEIGHT	240
#define D_FRAMERATE	30


/* Structure to contain all our information, so we can pass it to callbacks */
typedef struct _GstCustomData
{
  GstElement    *pipeline, *source, *decoder, *converter, *sink;
  GstElement    *decode_filter, *convert_filter;
  GstCaps       *decode_caps, *convert_caps;

  GMainLoop     *loop;

  GstBus	*bus;
  guint		bus_watch_id;

} GstCustomData;



// Callback function
static GstFlowReturn	on_new_sample(GstElement *sink);


// Gst functions
GstFlowReturn		create_gst_pipeline(GstCustomData *gstData, char *device);
void			run_gst_loop(GstCustomData *gstData, char *device);
void			clean_gst_pipeline(GstCustomData *gstData);

//
void			jiwy_execute(unsigned char *map_data);

#endif
