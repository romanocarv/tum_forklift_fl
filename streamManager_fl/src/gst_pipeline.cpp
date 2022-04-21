#include "gst_pipeline.hpp"
#include <gst/gst.h>
#include <gst/gst.h>
#include <tgmath.h>


int delta_height;
int delta_width;

//gst_pipeline::gst_pipeline(int *argc, char **argv[])
Gst_pipeline::Gst_pipeline(gint port, gchar *ip, int width, int height)
{
  //ToDo: Port & IP mit Konstruktorargument festlegen
  cData.port	 	    = port;
  cData.ipAddress  = ip;
  cData.width      = width;
  cData.height    = height;
  

  // Plugins einbinden
  GstRegistry *registry;
  registry = gst_registry_get();
  gst_registry_scan_path(registry, "/usr/local/lib/gstreamer-1.0");

}

void Gst_pipeline::create_pipeline(int camera_option)
{

 



  switch (camera_option){

    case 1: 
    case 2: 
    case 3:
    //TIS
    cData.source_factory = gst_element_factory_find ("tcambin");
    cData.sink_factory   = gst_element_factory_find ("udpsink");

    if (!cData.source_factory || !cData.sink_factory) {
      g_printerr ("Not all element factories could be created.\n");
      //return -1;
    }
    break;

    case 6:
    // FLIR USB
    cData.source_factory = gst_element_factory_find ("aravissrc");
    cData.sink_factory   = gst_element_factory_find ("udpsink");

    if (!cData.source_factory || !cData.sink_factory) {
      g_printerr ("Not all element factories could be created.\n");
      //return -1;
    }
    break;

    case 4: 
    //Webcam 1
    cData.source_factory = gst_element_factory_find ("v4l2src");
    cData.sink_factory   = gst_element_factory_find ("udpsink");

    if (!cData.source_factory || !cData.sink_factory) {
      g_printerr ("Not all element factories could be created.\n");
      //return -1;
    }
    break;

    case 5: 
    //Webcam 2
    cData.source_factory = gst_element_factory_find ("v4l2src");
    cData.sink_factory   = gst_element_factory_find ("udpsink");

    if (!cData.source_factory || !cData.sink_factory) {
      g_printerr ("Not all element factories could be created.\n");
      //return -1;
    }
    break;

    
  }




  // Kameraunabhängige Elemente

  cData.pipeline 	  = gst_pipeline_new ("pipeline");
  cData.sink 		    = gst_element_factory_make ("udpsink", "sink");
  cData.videorate 		= gst_element_factory_make ("videorate", "videorate");
  cData.videoscale   = gst_element_factory_make ("videoscale", "videoscale");
  cData.videocrop 		= gst_element_factory_make ("videocrop", "videocrop");
  cData.textoverlay  = gst_element_factory_make ("textoverlay", "textoverlay");
  cData.videoflip = gst_element_factory_make ("videoflip", "videoflip");
  cData.gdkpixbufoverlay = gst_element_factory_make ("gdkpixbufoverlay", "overlaytool");


switch (camera_option){

 
    case 1: 
      //TIS Vorne
      cData.source 	  = gst_element_factory_make ("tcambin", "source");
      cData.filter	= gst_element_factory_make("capsfilter","filter");
      cData.scale_filter	= gst_element_factory_make("capsfilter","capsfilter");
      cData.rtppay	= gst_element_factory_make("rtph264pay","rtppay");
      cData.codec = gst_element_factory_make("omxh264enc", "codec");
      cData.parse = gst_element_factory_make("h264parse", "h264parse");
      cData.gdppay = gst_element_factory_make("gdppay", "gdppay");
      cData.video_convert = gst_element_factory_make ("nvvidconv", "video_convert");
      cData.video_caps 	= gst_caps_new_simple ("video/x-raw",
        "format" , G_TYPE_STRING, "BGRx",
        "width", G_TYPE_INT, 1280,
        "height", G_TYPE_INT, 720,
        "framerate", GST_TYPE_FRACTION, 30, 1,
        NULL); 

      g_object_set (G_OBJECT (cData.filter), "caps", cData.video_caps, NULL);
      gst_caps_unref (cData.video_caps);
      
    
      cData.scale_caps 	= gst_caps_new_simple ("video/x-raw",
        "width", G_TYPE_INT, cData.width,
        "height", G_TYPE_INT, cData.height,
        NULL); 

      g_object_set (G_OBJECT (cData.scale_filter), "caps", cData.scale_caps, NULL);
      gst_caps_unref (cData.scale_caps);



      delta_width = floor(floor((1280 - cData.width) / 2 ) / 2) * 2;
      delta_height = floor(floor((720 - cData.height) / 2 ) / 2) * 2;

      g_object_set(G_OBJECT(cData.videocrop),"top", delta_height, NULL);
      g_object_set(G_OBJECT(cData.videocrop),"left",delta_width , NULL);
      g_object_set(G_OBJECT(cData.videocrop),"right", delta_width, NULL);
      g_object_set(G_OBJECT(cData.videocrop),"bottom", delta_height, NULL);
      

      g_object_set(G_OBJECT(cData.source),"serial", "05220229", NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"text", "FRONT", NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"halignment", 0, NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"valignment", 2, NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"font-desc", "Ubuntu Mono 12pt", NULL);

      g_object_set(cData.gdkpixbufoverlay, "location", "/home/enabl-tech/tum_ws/src/tum_forklift/gui/src/wheel2.png", NULL);
      g_object_set(cData.gdkpixbufoverlay, "offset-x", 100, NULL);
      g_object_set(cData.gdkpixbufoverlay, "offset-y", 100, NULL);
      
     g_object_set(G_OBJECT(cData.source),"device-caps", "video/x-bayer,format=grbg16,width=1920,height=1080,framerate=30/1", NULL);



      //g_object_set(G_OBJECT(cData.source),"bottom", delta_height, NULL);

      //g_object_set(G_OBJECT(cData.source),"async-handling", true, NULL);

      break;

    case 2: 
      //TIS Links
      cData.source 	  = gst_element_factory_make ("tcambin", "source");
      cData.filter	= gst_element_factory_make("capsfilter","filter");
      cData.scale_filter	= gst_element_factory_make("capsfilter","capsfilter");
      cData.rtppay	= gst_element_factory_make("rtph264pay","rtppay");
      cData.codec = gst_element_factory_make("omxh264enc", "codec");
      cData.parse = gst_element_factory_make("h264parse", "h264parse");
      cData.gdppay = gst_element_factory_make("gdppay", "gdppay");
      cData.video_convert = gst_element_factory_make ("nvvidconv", "video_convert");
      cData.video_caps 	= gst_caps_new_simple ("video/x-raw",
        "format" , G_TYPE_STRING, "BGRx",
        "width", G_TYPE_INT, 640,
        "height", G_TYPE_INT, 480,
        "framerate", GST_TYPE_FRACTION, 30, 1,
        NULL); 

      g_object_set (G_OBJECT (cData.filter), "caps", cData.video_caps, NULL);
      gst_caps_unref (cData.video_caps);
      
    
      cData.scale_caps 	= gst_caps_new_simple ("video/x-raw",
        "width", G_TYPE_INT, cData.width,
        "height", G_TYPE_INT, cData.height,
        NULL); 

      g_object_set (G_OBJECT (cData.scale_filter), "caps", cData.scale_caps, NULL);
      gst_caps_unref (cData.scale_caps);



      delta_width = floor(floor((640 - cData.width) / 2 ) / 2) * 2;
      delta_height = floor(floor((480 - cData.height) / 2 ) / 2) * 2;

      g_object_set(G_OBJECT(cData.videocrop),"top", delta_height, NULL);
      g_object_set(G_OBJECT(cData.videocrop),"left",delta_width , NULL);
      g_object_set(G_OBJECT(cData.videocrop),"right", delta_width, NULL);
      g_object_set(G_OBJECT(cData.videocrop),"bottom", delta_height, NULL);
      

      g_object_set(G_OBJECT(cData.source),"serial", "05220231", NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"text", "LEFT", NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"halignment", 0, NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"valignment", 2, NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"font-desc", "Ubuntu Mono 12pt", NULL);
 
      g_object_set(G_OBJECT(cData.source),"device-caps", "video/x-bayer,format=grbg16,width=1920,height=1080,framerate=30/1", NULL);

      

      //g_object_set(G_OBJECT(cData.source),"bottom", delta_height, NULL);

      //g_object_set(G_OBJECT(cData.source),"async-handling", true, NULL);

      break;
    case 3:
      //TIS Rechts
      cData.source 	  = gst_element_factory_make ("tcambin", "source");
      cData.filter	= gst_element_factory_make("capsfilter","filter");
      cData.scale_filter	= gst_element_factory_make("capsfilter","capsfilter");
      cData.rtppay	= gst_element_factory_make("rtph264pay","rtppay");
      cData.codec = gst_element_factory_make("omxh264enc", "codec");
      cData.parse = gst_element_factory_make("h264parse", "h264parse");
      cData.gdppay = gst_element_factory_make("gdppay", "gdppay");
      cData.video_convert = gst_element_factory_make ("nvvidconv", "video_convert");
      cData.video_caps 	= gst_caps_new_simple ("video/x-raw",
        "format" , G_TYPE_STRING, "BGRx",
        "width", G_TYPE_INT, 640,
        "height", G_TYPE_INT, 480,
        "framerate", GST_TYPE_FRACTION, 30, 1,
        NULL); 

      g_object_set (G_OBJECT (cData.filter), "caps", cData.video_caps, NULL);
      gst_caps_unref (cData.video_caps);
      
    
      cData.scale_caps 	= gst_caps_new_simple ("video/x-raw",
        "width", G_TYPE_INT, cData.width,
        "height", G_TYPE_INT, cData.height,
        NULL); 

      g_object_set (G_OBJECT (cData.scale_filter), "caps", cData.scale_caps, NULL);
      gst_caps_unref (cData.scale_caps);



      delta_width = floor(floor((640 - cData.width) / 2 ) / 2) * 2;
      delta_height = floor(floor((480 - cData.height) / 2 ) / 2) * 2;

      g_object_set(G_OBJECT(cData.videocrop),"top", delta_height, NULL);
      g_object_set(G_OBJECT(cData.videocrop),"left",delta_width , NULL);
      g_object_set(G_OBJECT(cData.videocrop),"right", delta_width, NULL);
      g_object_set(G_OBJECT(cData.videocrop),"bottom", delta_height, NULL);
      

      g_object_set(G_OBJECT(cData.source),"serial", "05220232", NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"text", "RIGHT", NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"halignment", 0, NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"valignment", 2, NULL);
      g_object_set(G_OBJECT(cData.textoverlay),"font-desc", "Ubuntu Mono 12pt", NULL);
      g_object_set(G_OBJECT(cData.source),"device-caps", "video/x-bayer,format=grbg16,width=1920,height=1080,framerate=30/1", NULL);



      //g_object_set(G_OBJECT(cData.source),"bottom", delta_height, NULL);

      //g_object_set(G_OBJECT(cData.source),"async-handling", true, NULL);

      break;

       
  case 6:
  //FLIR USB
    cData.source 	  = gst_element_factory_make ("aravissrc", "source");
    cData.filter	= gst_element_factory_make("capsfilter","filter");
    //cData.scale_filter	= gst_element_factory_make("capsfilter","capsfilter");
    cData.rtppay	= gst_element_factory_make("rtph264pay","rtppay");
    cData.codec = gst_element_factory_make("omxh264enc", "codec");
    cData.parse = gst_element_factory_make("h264parse", "h264parse");
    cData.gdppay = gst_element_factory_make("gdppay", "gdppay");
    cData.video_convert = gst_element_factory_make ("nvvidconv", "video_convert");
    cData.video_caps 	= gst_caps_new_simple ("video/x-raw",
      "format", G_TYPE_STRING, "UYVY",
      "width", G_TYPE_INT, 1280,
      "height", G_TYPE_INT, 720,
      "framerate", GST_TYPE_FRACTION, 30, 1,
      NULL); 
    g_object_set(G_OBJECT(cData.source),"camera-name", "FLIR-0131F37E", NULL);
    g_object_set (G_OBJECT (cData.filter), "caps", cData.video_caps, NULL);
    gst_caps_unref (cData.video_caps);

    //cData.scale_caps 	= gst_caps_new_simple ("video/x-raw",
      //"width", G_TYPE_INT, cData.width,
      //"height", G_TYPE_INT, cData.height,
      //NULL); 

    //g_object_set (G_OBJECT (cData.scale_filter), "caps", cData.scale_caps, NULL);
    //gst_caps_unref (cData.scale_caps);


    delta_width = floor(floor((1280 - cData.width) / 2 ) / 2) * 2;
    delta_height = floor(floor((720 - cData.height) / 2 ) / 2) * 2;

    g_object_set(G_OBJECT(cData.videocrop),"top", delta_height, NULL);
    g_object_set(G_OBJECT(cData.videocrop),"left",delta_width , NULL);
    g_object_set(G_OBJECT(cData.videocrop),"right", delta_width, NULL);
    g_object_set(G_OBJECT(cData.videocrop),"bottom", delta_height, NULL);

    g_object_set(G_OBJECT(cData.textoverlay),"text", "BACK", NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"halignment", 0, NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"valignment", 2, NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"font-desc", "Ubuntu Mono 12pt", NULL);
    break;




  case 4:
  //webcam 1 Gabel
    cData.source 	  = gst_element_factory_make ("v4l2src", "source");
    cData.rtppay	= gst_element_factory_make("rtph264pay","rtppay");
    cData.filter	= gst_element_factory_make("capsfilter","filter");
    //cData.scale_filter	= gst_element_factory_make("capsfilter","capsfilter");
    cData.jpegcodec = gst_element_factory_make("jpegdec", "jpegcodec");
    cData.codec = gst_element_factory_make("omxh264enc", "codec");
    cData.parse = gst_element_factory_make("h264parse", "h264parse");
    cData.jpegparse = gst_element_factory_make("jpegparse", "jpegparse");
    cData.gdppay = gst_element_factory_make("gdppay", "gdppay");
    cData.video_convert = gst_element_factory_make ("videoconvert", "video_convert");
    cData.video_caps 	= gst_caps_new_simple ("image/jpeg",
      "framerate", GST_TYPE_FRACTION, 30, 1,
      "width", G_TYPE_INT, 1280,
      "height", G_TYPE_INT, 720,
      NULL); 

    g_object_set (G_OBJECT (cData.filter), "caps", cData.video_caps, NULL);
    gst_caps_unref (cData.video_caps);
/*
    cData.scale_caps 	= gst_caps_new_simple ("video/x-raw",
      "width", G_TYPE_INT, cData.width,
      "height", G_TYPE_INT, cData.height,
      NULL); 

    g_object_set (G_OBJECT (cData.scale_filter), "caps", cData.scale_caps, NULL);
    gst_caps_unref (cData.scale_caps);
*/

    
   //g_object_set(G_OBJECT(cData.source),"v4l2.device.card", "CyberTrack\ H7", NULL);
     g_object_set(G_OBJECT(cData.source),"device", "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._CyberTrack_H7_SN0001-video-index0", NULL);

    delta_width = floor(floor((1280 - cData.width) / 2 ) / 2) * 2;
    delta_height = floor(floor((720 - cData.height) / 2 ) / 2) * 2;

    g_object_set(G_OBJECT(cData.videocrop),"top", delta_height, NULL);
    g_object_set(G_OBJECT(cData.videocrop),"left",delta_width , NULL);
    g_object_set(G_OBJECT(cData.videocrop),"right", delta_width, NULL);
    g_object_set(G_OBJECT(cData.videocrop),"bottom", delta_height, NULL);

    g_object_set(G_OBJECT(cData.textoverlay),"text", "BACK", NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"halignment", 0, NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"valignment", 2, NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"font-desc", "Ubuntu Mono 12pt", NULL);
    break;

  case 5:
  //webcam 2 Fork
    cData.source 	  = gst_element_factory_make ("v4l2src", "source");
    cData.rtppay	= gst_element_factory_make("rtph264pay","rtppay");
    cData.filter	= gst_element_factory_make("capsfilter","filter");
    //cData.scale_filter	= gst_element_factory_make("capsfilter","capsfilter");
    cData.jpegcodec = gst_element_factory_make("jpegdec", "jpegcodec");
    cData.codec = gst_element_factory_make("omxh264enc", "codec");
    cData.parse = gst_element_factory_make("h264parse", "h264parse");
    cData.jpegparse = gst_element_factory_make("jpegparse", "jpegparse");
    cData.gdppay = gst_element_factory_make("gdppay", "gdppay");
    cData.video_convert = gst_element_factory_make ("videoconvert", "video_convert");
    cData.video_caps 	= gst_caps_new_simple ("image/jpeg",
      "framerate", GST_TYPE_FRACTION, 30, 1,
      "width", G_TYPE_INT, 1280,
      "height", G_TYPE_INT, 720,
      NULL); 

    g_object_set (G_OBJECT (cData.filter), "caps", cData.video_caps, NULL);
    gst_caps_unref (cData.video_caps);
/*
    cData.scale_caps 	= gst_caps_new_simple ("video/x-raw",
      "width", G_TYPE_INT, cData.width,
      "height", G_TYPE_INT, cData.height,
      NULL); 

    g_object_set (G_OBJECT (cData.scale_filter), "caps", cData.scale_caps, NULL);
    gst_caps_unref (cData.scale_caps);
*/

    
   //g_object_set(G_OBJECT(cData.source),"v4l2.device.card", "CyberTrack\ H7", NULL);
   g_object_set(G_OBJECT(cData.source),"device", "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0", NULL);

    delta_width = floor(floor((1280 - cData.width) / 2 ) / 2) * 2;
    delta_height = floor(floor((720 - cData.height) / 2 ) / 2) * 2;

    g_object_set(G_OBJECT(cData.videocrop),"top", delta_height, NULL);
    g_object_set(G_OBJECT(cData.videocrop),"left",delta_width , NULL);
    g_object_set(G_OBJECT(cData.videocrop),"right", delta_width, NULL);
    g_object_set(G_OBJECT(cData.videocrop),"bottom", delta_height, NULL);

    g_object_set(G_OBJECT(cData.textoverlay),"text", "FORK", NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"halignment", 0, NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"valignment", 2, NULL);
    g_object_set(G_OBJECT(cData.textoverlay),"font-desc", "Ubuntu Mono 12pt", NULL);
    break;
}


  /* Modify the properties */
  g_object_set(G_OBJECT(cData.sink),"port", cData.port, NULL);
  g_object_set(G_OBJECT(cData.sink),"host", cData.ipAddress, NULL);
  g_object_set(G_OBJECT(cData.codec),"bitrate", 40000000, NULL);
  g_object_set(G_OBJECT(cData.codec),"preset-level", "UltraFastPreset", NULL);
  g_object_set(G_OBJECT(cData.rtppay),"config-interval", 1, NULL);
  g_object_set(G_OBJECT(cData.rtppay),"pt", 96, NULL);
  g_object_set(G_OBJECT(cData.videoflip),"method", 2, NULL);
  g_object_set(G_OBJECT(cData.textoverlay),"auto-resize", true, NULL);
  //g_object_set(G_OBJECT(cData.sink),"sync", false, NULL);
  //g_object_set(G_OBJECT(cData.source),"async-handling", true, NULL);


  //set video source
  g_object_set(G_OBJECT (cData.source), "src", NULL);
  g_printerr ("==>Set video source.\n");
  g_object_set(G_OBJECT (cData.sink), "sync", false, NULL);
  g_printerr ("==>Set video sink.\n");

  /* Create the empty pipeline */
  cData.pipeline = gst_pipeline_new ("test-pipeline");

  /* Check for NULL Objects*/
  if (!cData.pipeline || !cData.source || !cData.sink ||!cData.rtppay || !cData.filter) {
    g_printerr ("Not all elements could be created.\n");
    //return -1;
  }


switch (camera_option){
/*
  case 1: 
  case 2: 
  case 3:
  // Pipeline für TIS
  gst_bin_add_many (GST_BIN (cData.pipeline), cData.source, cData.filter, cData.video_convert, cData.codec, cData.parse, cData.rtppay, cData.gdppay,cData.sink, NULL);
  if (gst_element_link_many (cData.source, cData.filter, cData.video_convert,cData.codec , cData.parse, cData.rtppay, cData.gdppay, cData.sink, NULL) != TRUE) {
    g_printerr ("TIS Elements could not be linked.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }
  break;
*/

    case 1: 
  case 2: 
  case 3:
  // Pipeline für TIS
  gst_bin_add_many (GST_BIN (cData.pipeline), cData.source, cData.videoscale, cData.scale_filter, cData.textoverlay, cData.video_convert, cData.codec, cData.parse, cData.rtppay, cData.gdppay,cData.sink, NULL);
  if (gst_element_link_many (cData.source, cData.videoscale, cData.scale_filter, cData.textoverlay, cData.video_convert,cData.codec , cData.parse, cData.rtppay, cData.gdppay, cData.sink, NULL) != TRUE) {
    g_printerr ("TIS Elements could not be linked.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }
  break;

  /*
    // Pipeline für TIS
  gst_bin_add_many (GST_BIN (cData.pipeline), cData.source, cData.videoscale, cData.filter, cData.videocrop, cData.textoverlay, cData.video_convert, cData.codec, cData.parse, cData.rtppay, cData.gdppay,cData.sink, NULL);
  if (gst_element_link_many (cData.source, cData.videoscale, cData.filter, cData.videocrop, cData.textoverlay, cData.video_convert,cData.codec , cData.parse, cData.rtppay, cData.gdppay, cData.sink, NULL) != TRUE) {
    g_printerr ("TIS Elements could not be linked.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }
  break;
  */

  

  case 6:
  // Pipeline für FLIR USB
  gst_bin_add_many (GST_BIN (cData.pipeline), cData.source, cData.videorate, cData.filter, cData.videocrop, cData.video_convert, cData.codec, cData.parse, cData.rtppay, cData.gdppay,cData.sink, NULL);
  if (gst_element_link_many (cData.source, cData.filter, cData.videorate, cData.videocrop , cData.video_convert, cData.codec, cData.parse, cData.rtppay, cData.gdppay,cData.sink, NULL) != TRUE) {
    g_printerr ("FLIR Elements could not be linked.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }
  break;
  

    case 4:
    case 5:
  // Pipeline für Webcam
  gst_bin_add_many (GST_BIN (cData.pipeline), cData.source, cData.filter, cData.jpegparse, cData.jpegcodec, cData.videocrop, cData.textoverlay, cData.video_convert, cData.codec, cData.parse, cData.rtppay, cData.gdppay,cData.sink, NULL);
  if (gst_element_link_many (cData.source, cData.filter, cData.jpegparse, cData.jpegcodec, cData.videocrop, cData.textoverlay, cData.video_convert, cData.codec, cData.parse, cData.rtppay, cData.gdppay,cData.sink, NULL) != TRUE) {
    g_printerr ("TIS Elements could not be linked.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }
  break;

read_bus_pipeline();
}

}

void Gst_pipeline::start_pipeline()
{
  /* Start playing */
  cData.ret = gst_element_set_state (cData.pipeline, GST_STATE_PLAYING);
  if (cData.ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the playing state.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }

}

void Gst_pipeline::pause_pipeline()
{
  /* Start playing */
  cData.ret = gst_element_set_state (cData.pipeline, GST_STATE_PAUSED);
  if (cData.ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the pausing state.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }
}
void Gst_pipeline::ready_pipeline()
{
  /* Start playing */
  cData.ret = gst_element_set_state (cData.pipeline, GST_STATE_READY);
  if (cData.ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the pausing state.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }
}


void Gst_pipeline::kill_pipeline()
{
  /* Start playing */
  cData.ret = gst_element_set_state (cData.pipeline, GST_STATE_NULL);
  if (cData.ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the pausing state.\n");
    gst_object_unref (cData.pipeline);
    //return -1;
  }
}

void Gst_pipeline::read_bus_pipeline()
{
  /* Wait until error or EOS */
  cData.bus = gst_element_get_bus (cData.pipeline);
  cData.msg =
      gst_bus_timed_pop_filtered (cData.bus, GST_CLOCK_TIME_NONE,
      static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  
}

void Gst_pipeline::set_ip(gchar* ip){
cData.ipAddress = ip;

//prüfen ob pipeline neu gebaut werden muss


}

Gst_pipeline::~Gst_pipeline()
{
  /* Free resources */
  if (cData.msg != NULL)
    gst_message_unref (cData.msg);
  gst_object_unref (cData.bus);
  gst_element_set_state (cData.pipeline, GST_STATE_NULL);
  gst_object_unref (cData.pipeline);
  //return 0;
}



/*

int main (int arg, char *argv[])
{
  g_printerr("main");
  // Initialize GStreamer 
  // ToDo add argc, argv
  gst_init (&arg, &argv);
  gst_pipeline test_pipe(&arg, &argv);
  
}
*/

