#include <iostream>
#include <gst/gst.h>
#include <gst/gst.h>


/*  
Camera IDs:

TIS vorne: 05220229
TIS links: 05220232
TIS rechts: 05220231
FLIR USB: FLIR-0131F37E
FLIR GIGE:
webcam: /dev/video6
*/



typedef struct CustomData{
        //GstElement *pipeline, *source, *sink, *video_convert, *rtppay, *filter, *jpegcodec, *scale_filter, *gdppay, *dec, *codec, *parse, *jpegparse, *videorate, *videoscale, *videocrop, *textoverlay, *videoflip;
	GstElement *pipeline, *source, *sink, *video_convert, *rtppay, *filter, *jpegcodec, *scale_filter, *gdppay, *dec, *codec, *parse, *jpegparse, *videorate, *videoscale, *videocrop, *textoverlay, *videoflip, *gdkpixbufoverlay;
        GstCaps *video_caps, *scale_caps;
        GstElementFactory *source_factory, *sink_factory;
        GstBus *bus;
        GstMessage *msg;
        GstStateChangeReturn ret;
        gboolean terminate;
        gint	  	port;
        gchar *ipAddress;
        int width, height;
    } ;

class Gst_pipeline
{
  
public:
    //gst_pipeline(int *argc, char **argv[]);
    Gst_pipeline(gint port, gchar *ip, int width, int height);
    ~Gst_pipeline();
    //void create_pipeline(int *argc, char **argv[]);
    void set_ip(gchar* ip="192.168.178.44");
    void create_pipeline(int camera_option); 
    void start_pipeline();
    void pause_pipeline();
    void kill_pipeline();
    void ready_pipeline();
    void read_bus_pipeline();
    
    CustomData cData;


private:


};
