/*
 *  Camera Listup Program Ver.1.03
 *  (c)2011 saibara
 *
 *  Using V4l2 Video Capture Example
 *
 *  Ver.1.00 2011/03/07 make first
 *  Ver.1.01 2011/03/10 add VIDIOC_QUERYCAP
 *  Ver.1.02 2011/03/14 rebuild capacity flag display
 *  Ver.1.03 2011/03/14 add VIDIOC_QUERYCTRL
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>

/* Define TRUE,FALSE */
#ifndef TRUE
#define TRUE    1
#define FALSE   0
#endif

static char *           dev_name        = NULL;
static int              fd              = -1;
static int		v_flag		= FALSE;

static void errno_exit(const char *s)
{
  fprintf(stderr, "%s error %d, %s\n",s, errno, strerror(errno));  
  exit(EXIT_FAILURE);
}

static int xioctl(int fd,int request,void *arg)
{
  int r;
  do{ r = ioctl(fd, request, arg); }
  while(-1 == r && EINTR == errno);
  return r;
}

struct id_list {
  __u32 id;
  char  *name;
};

static int search_id_number(struct id_list list[],__u32 id)
{
  int i;
  i = 0;
  while(list[i].name != NULL)
    {
      if(list[i].id == id) return(i);
      i++;
    }
  return(-1);
}

static char id_unknown[] = "Unknown...";

static char *get_id_name(struct id_list list[],__u32 id)
{
  int i;
  i = search_id_number(list,id);
  if(i >= 0)
    {
      return(list[i].name);
    }
  return(id_unknown);
}

/* ------------------------------------------------------------------ */
static void close_device(void)
{
  if(-1 == close(fd)) errno_exit("close");
  fd = -1;
}

static void open_device(void)
{
  struct stat st; 
  
  if(-1 == stat(dev_name, &st))
    {
      fprintf(stderr, "Cannot identify '%s': %d, %s\n",
	      dev_name, errno, strerror(errno));
      exit(EXIT_FAILURE);
    }
  if(!S_ISCHR(st.st_mode))
    {
      fprintf(stderr, "%s is no device\n", dev_name);
      exit(EXIT_FAILURE);
    }
  fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
  if(-1 == fd)
    {
      fprintf(stderr, "Cannot open '%s': %d, %s\n",
	      dev_name, errno, strerror(errno));
      exit(EXIT_FAILURE);
    }
}

// listup
// http://yasu-2.blogspot.com/2010/04/v4l2.html
static void list_frameintervals(__u32 pixel_format,__u32 width,__u32 height)
{
  int i;
  struct v4l2_frmivalenum fmt;
  fmt.index = i = 0;
  fmt.pixel_format = pixel_format;
  fmt.width = width;
  fmt.height = height;
  
  printf("    ");
  while(-1 != xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fmt))
    {
      // printf("    %i: ", fmt.index);
      switch(fmt.type)
	{
	case V4L2_FRMIVAL_TYPE_DISCRETE:
	  printf(" %d/%d",fmt.discrete.numerator,fmt.discrete.denominator);
	  break;
	case V4L2_FRMIVAL_TYPE_CONTINUOUS: // ???
	  printf(" %d/%d-%d/%d",
		 fmt.stepwise.min.numerator,fmt.stepwise.min.denominator,
		 fmt.stepwise.max.numerator,fmt.stepwise.max.denominator);
	  break;
	case V4L2_FRMIVAL_TYPE_STEPWISE: // ???
	  printf(" %d/%d-%d/%dstep%d/%d",
		 fmt.stepwise.min.numerator,fmt.stepwise.min.denominator,
		 fmt.stepwise.max.numerator,fmt.stepwise.max.denominator,
		 fmt.stepwise.step.numerator,fmt.stepwise.step.denominator);
	  break;
	}
      memset(&fmt, 0, sizeof(struct v4l2_frmivalenum));
      fmt.index = ++i;
      fmt.pixel_format = pixel_format;
      fmt.width = width;
      fmt.height = height;
    }
  printf("\n");
}

static void list_framesizes(__u32 pixel_format)
{
  int i;
  struct v4l2_frmsizeenum fmt;
  fmt.index = i = 0;
  fmt.pixel_format = pixel_format;
  
  while(-1 != xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fmt))
    {
      switch(fmt.type)
	{
	case V4L2_FRMSIZE_TYPE_DISCRETE:
	  printf("  %i: (%d x %d)", fmt.index,
		 fmt.discrete.width,fmt.discrete.height);
	  break;
	default:
	  // V4L2_FRMSIZE_TYPE_CONTINUOUS2Continuous frame size.
	  // V4L2_FRMSIZE_TYPE_STEPWISE3Step-wise defined frame size.
	  printf("  %i: Unknown TYPE\n", fmt.index);
	}
      list_frameintervals(pixel_format,
			  fmt.discrete.width,fmt.discrete.height);
      memset(&fmt, 0, sizeof(struct v4l2_frmsizeenum));
      fmt.index = ++i;
      fmt.pixel_format = pixel_format;
    }
}

static void list_formats(void)
{
  int i;
  struct v4l2_fmtdesc fmt;
  fmt.index = i = 0;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  while(-1 != xioctl(fd, VIDIOC_ENUM_FMT, &fmt))
    {
      printf("Image format #%i : %c%c%c%c (%s)\n", fmt.index,
	     fmt.pixelformat >> 0, fmt.pixelformat >> 8,
	     fmt.pixelformat >> 16, fmt.pixelformat >> 24, fmt.description);
      list_framesizes(fmt.pixelformat);
      memset(&fmt, 0, sizeof(struct v4l2_fmtdesc));
      fmt.index = ++i;
      fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }
}

struct id_list cap_list[] = {
{V4L2_CAP_VIDEO_CAPTURE,"V4L2_CAP_VIDEO_CAPTURE (Video Capture)"},
{V4L2_CAP_VIDEO_OUTPUT,"V4L2_CAP_VIDEO_OUTPUT (Video Output)"},
{V4L2_CAP_VIDEO_OVERLAY,"V4L2_CAP_VIDEO_OVERLAY (Video Overlay)"},
{V4L2_CAP_VBI_CAPTURE,"V4L2_CAP_VBI_CAPTURE (Raw VBI Capture)"},
{V4L2_CAP_VBI_OUTPUT,"V4L2_CAP_VBI_OUTPUT (Raw VBI Output)"},
{V4L2_CAP_SLICED_VBI_CAPTURE,
"V4L2_CAP_SLICED_VBI_CAPTURE (Sliced VBI Capture)"},
{V4L2_CAP_SLICED_VBI_OUTPUT,
"V4L2_CAP_SLICED_VBI_OUTPUT (Sliced VBI Output)"},
{V4L2_CAP_RDS_CAPTURE,"V4L2_CAP_RDS_CAPTURE (Undefined.[to be defined])"},
{V4L2_CAP_VIDEO_OUTPUT_OVERLAY,
"V4L2_CAP_VIDEO_OUTPUT_OVERLAY (Video Output Overlay (OSD))"},
{V4L2_CAP_TUNER,"V4L2_CAP_TUNER (Tuner)"},
{V4L2_CAP_AUDIO,"V4L2_CAP_AUDIO (Audio inputs or outputs)"},
{V4L2_CAP_RADIO,"V4L2_CAP_RADIO (Radio receiver)"},
{V4L2_CAP_READWRITE,"V4L2_CAP_READWRITE (Read/write() I/O method)"},
{V4L2_CAP_ASYNCIO,"V4L2_CAP_ASYNCIO (Asynchronous I/O method)"},
{V4L2_CAP_STREAMING,"V4L2_CAP_STREAMING (Streaming I/O method)"},
{0,NULL}};

void disp_capabilities(__u32 flag)
{
  struct id_list *cap;

  cap = cap_list;
  while(cap->name != NULL)
    {
      printf("%60s : %s\n",
	     cap->name,((cap->id&flag)?"OK":"Not supported."));
      cap++;
    }
}

void query_capabilities(void)
{
  struct v4l2_capability fmt;
  if(-1 != xioctl(fd, VIDIOC_QUERYCAP, &fmt))
    {
      printf("Driver name     : %s\n",fmt.driver);
      printf("Driver Version  : %u.%u.%u\n",
        (fmt.version >> 16) & 0xFF,
        (fmt.version >> 8) & 0xFF,
         fmt.version & 0xFF);
      printf("Device name     : %s\n",fmt.card);
      printf("Bus information : %s\n",fmt.bus_info);
      printf("Capabilities    : %08xh\n",fmt.capabilities);
      disp_capabilities(fmt.capabilities);
      printf("\n");
    }
}

void disp_flags(__u32 flags,__u32 flag,char list[])
{
  if(flags&flag)
    {
      printf("\t\t%s\n",list);
    }
}

static void enumerate_menu(struct v4l2_queryctrl *queryctrl)
{
  struct v4l2_querymenu querymenu;
  
  printf ("\t\t  Menu items:\n");
  memset (&querymenu, 0, sizeof (querymenu));
  querymenu.id = queryctrl->id;
  for(querymenu.index = queryctrl->minimum;
      querymenu.index <= queryctrl->maximum;
      querymenu.index++)
    {
      if(0 == ioctl (fd, VIDIOC_QUERYMENU, &querymenu))
	{
	  printf ("\t\t  %d : %s\n", querymenu.index,querymenu.name);
	}
      else
	{
	  perror ("VIDIOC_QUERYMENU");
	  exit (EXIT_FAILURE);
	}
    }
}

struct id_list query_id_list[] = {
{V4L2_CID_BRIGHTNESS,"V4L2_CID_BRIGHTNESS"},
{V4L2_CID_CONTRAST,"V4L2_CID_CONTRAST"},
{V4L2_CID_SATURATION,"V4L2_CID_SATURATION"},
{V4L2_CID_HUE,"V4L2_CID_HUE"},

{V4L2_CID_AUDIO_VOLUME,"V4L2_CID_AUDIO_VOLUME"},
{V4L2_CID_AUDIO_BALANCE,"V4L2_CID_AUDIO_BALANCE"},
{V4L2_CID_AUDIO_BASS,"V4L2_CID_AUDIO_BASS"},
{V4L2_CID_AUDIO_TREBLE,"V4L2_CID_AUDIO_TREBLE"},
{V4L2_CID_AUDIO_MUTE,"V4L2_CID_AUDIO_MUTE"},
{V4L2_CID_AUDIO_LOUDNESS,"V4L2_CID_AUDIO_LOUDNESS"},
{V4L2_CID_BLACK_LEVEL,"V4L2_CID_BLACK_LEVEL(Deprecated)"},

{V4L2_CID_AUTO_WHITE_BALANCE,"V4L2_CID_AUTO_WHITE_BALANCE"},
{V4L2_CID_DO_WHITE_BALANCE,"V4L2_CID_DO_WHITE_BALANCE"},
{V4L2_CID_RED_BALANCE,"V4L2_CID_RED_BALANCE"},
{V4L2_CID_BLUE_BALANCE,"V4L2_CID_BLUE_BALANCE"},
{V4L2_CID_GAMMA,"V4L2_CID_GAMMA"},
{V4L2_CID_WHITENESS,"V4L2_CID_WHITENESS(Deprecated)"},

{V4L2_CID_EXPOSURE,"V4L2_CID_EXPOSURE"},
{V4L2_CID_AUTOGAIN,"V4L2_CID_AUTOGAIN"},
{V4L2_CID_GAIN,"V4L2_CID_GAIN"},
{V4L2_CID_HFLIP,"V4L2_CID_HFLIP"},
{V4L2_CID_VFLIP,"V4L2_CID_VFLIP"},

//{V4L2_CID_HCENTER,"V4L2_CID_HCENTER(Deprecated)"},
//{V4L2_CID_VCENTER,"V4L2_CID_VCENTER(Deprecated)"},

{V4L2_CID_POWER_LINE_FREQUENCY,"V4L2_CID_POWER_LINE_FREQUENCY"},
{V4L2_CID_HUE_AUTO,"V4L2_CID_HUE_AUTO"},
{V4L2_CID_WHITE_BALANCE_TEMPERATURE,"V4L2_CID_WHITE_BALANCE_TEMPERATURE"},
{V4L2_CID_SHARPNESS,"V4L2_CID_SHARPNESS"},
{V4L2_CID_BACKLIGHT_COMPENSATION,"V4L2_CID_BACKLIGHT_COMPENSATION"},
{V4L2_CID_CHROMA_AGC,"V4L2_CID_CHROMA_AGC"},
{V4L2_CID_COLOR_KILLER,"V4L2_CID_COLOR_KILLER"},
{V4L2_CID_COLORFX,"V4L2_CID_COLORFX"},
{V4L2_CID_AUTOBRIGHTNESS,"V4L2_CID_AUTOBRIGHTNESS"},
{V4L2_CID_BAND_STOP_FILTER,"V4L2_CID_BAND_STOP_FILTER"},
{V4L2_CID_ROTATE,"V4L2_CID_ROTATE"},
{V4L2_CID_BG_COLOR,"V4L2_CID_BG_COLOR"},
{V4L2_CID_CHROMA_GAIN,"V4L2_CID_CHROMA_GAIN"},

/* -------------------------------------------------------- */
{V4L2_CID_EXPOSURE_AUTO,"V4L2_CID_EXPOSURE_AUTO"},
{V4L2_CID_EXPOSURE_ABSOLUTE,"V4L2_CID_EXPOSURE_ABSOLUTE"},
{V4L2_CID_EXPOSURE_AUTO_PRIORITY,"V4L2_CID_EXPOSURE_AUTO_PRIORITY"},
{V4L2_CID_PAN_RELATIVE,"V4L2_CID_PAN_RELATIVE"},
{V4L2_CID_TILT_RELATIVE,"V4L2_CID_TILT_RELATIVE"},
{V4L2_CID_PAN_RESET,"V4L2_CID_PAN_RESET"},
{V4L2_CID_TILT_RESET,"V4L2_CID_TILT_RESET"},
{V4L2_CID_PAN_ABSOLUTE,"V4L2_CID_PAN_ABSOLUTE"},
{V4L2_CID_TILT_ABSOLUTE,"V4L2_CID_TILT_ABSOLUTE"},
{V4L2_CID_FOCUS_ABSOLUTE,"V4L2_CID_FOCUS_ABSOLUTE"},
{V4L2_CID_FOCUS_RELATIVE,"V4L2_CID_FOCUS_RELATIVE"},
{V4L2_CID_FOCUS_AUTO,"V4L2_CID_FOCUS_AUTO"},
{V4L2_CID_ZOOM_ABSOLUTE,"V4L2_CID_ZOOM_ABSOLUTE"},
{V4L2_CID_ZOOM_RELATIVE,"V4L2_CID_ZOOM_RELATIVE"},
{V4L2_CID_ZOOM_CONTINUOUS,"V4L2_CID_ZOOM_CONTINUOUS"},
{V4L2_CID_PRIVACY,"V4L2_CID_PRIVACY"},
{V4L2_CID_IRIS_ABSOLUTE,"V4L2_CID_IRIS_ABSOLUTE"},
{V4L2_CID_IRIS_RELATIVE,"V4L2_CID_IRIS_RELATIVE"},
// {,""},
{0,NULL}};

struct id_list ctrl_type_list[] = {
{V4L2_CTRL_TYPE_INTEGER,"V4L2_CTRL_TYPE_INTEGER"},
{V4L2_CTRL_TYPE_BOOLEAN,"V4L2_CTRL_TYPE_BOOLEAN"},
{V4L2_CTRL_TYPE_MENU,"V4L2_CTRL_TYPE_MENU"},
{V4L2_CTRL_TYPE_BUTTON,"V4L2_CTRL_TYPE_BUTTON"},
{V4L2_CTRL_TYPE_INTEGER64,"V4L2_CTRL_TYPE_INTEGER64"},
{V4L2_CTRL_TYPE_CTRL_CLASS,"V4L2_CTRL_TYPE_CTRL_CLASS"},
{V4L2_CTRL_TYPE_STRING,"V4L2_CTRL_TYPE_STRING"},
{0,NULL}};

static void disp_ctrl(__u32 id)
{
  struct v4l2_queryctrl qctrl;
  memset(&qctrl, 0, sizeof(struct v4l2_queryctrl));
  qctrl.id = id;
  if(-1 != xioctl(fd, VIDIOC_QUERYCTRL, &qctrl))
    {
      printf("Ctrl id(CID) : %08xh (%s)\n",qctrl.id,
	     get_id_name(query_id_list,qctrl.id));
      printf("\tCtrl name : %s\n",qctrl.name);
      printf("\tCtrl type : %d (%s)\n",qctrl.type,
	     get_id_name(ctrl_type_list,qctrl.type));
      if(qctrl.type == V4L2_CTRL_TYPE_MENU)
	{
	  enumerate_menu(&qctrl);
	}
      printf("\tMin,Max,Step,Default : %d,%d,%d,%d\n",
	     qctrl.minimum,qctrl.maximum,
	     qctrl.step,qctrl.default_value);
      printf("\tFlags    : %08xh\n",qctrl.flags);
      disp_flags(qctrl.flags,
		 V4L2_CTRL_FLAG_DISABLED,"V4L2_CTRL_FLAG_DISABLED");
      disp_flags(qctrl.flags,
		 V4L2_CTRL_FLAG_GRABBED,"V4L2_CTRL_FLAG_GRABBED");
      disp_flags(qctrl.flags,
		 V4L2_CTRL_FLAG_READ_ONLY,"V4L2_CTRL_FLAG_READ_ONLY");
      disp_flags(qctrl.flags,
		 V4L2_CTRL_FLAG_UPDATE,"V4L2_CTRL_FLAG_UPDATE");
      disp_flags(qctrl.flags,
		 V4L2_CTRL_FLAG_INACTIVE,"V4L2_CTRL_FLAG_INACTIVE");
      disp_flags(qctrl.flags,
		 V4L2_CTRL_FLAG_SLIDER,"V4L2_CTRL_FLAG_SLIDER");
    }
  else
    {
      if(v_flag)
	{
	  printf("Ctrl id(CID) : %08xh (%s)",qctrl.id,
		 get_id_name(query_id_list,qctrl.id));
	  // this ctrl id is not supported.
	    printf("\tUnsupported...\n");
	}
    }
}

static void query_ctrl(void)
{
  __u32 id;

  for(id=V4L2_CID_BASE;id<V4L2_CID_LASTP1;id++)
    {
      disp_ctrl(id);
    }

  // disp_ctrl(V4L2_CID_CAMERA_CLASS);
  
  for(id=V4L2_CID_CAMERA_CLASS_BASE+1;id<V4L2_CID_CAMERA_CLASS_BASE+19;id++)
    {
      disp_ctrl(id);
    }
  printf("\n");
}

int main (int argc,char **argv)
{
  int camera_no;
  char buf[1024];

  camera_no = 0;
  if(argc == 1)
    {
      printf("Usage : listup <camera no.> [<verbose flag>]\n");
      exit(0);
    }
  if(argc >= 2) camera_no = atoi(argv[1]);
  if(argc >= 3) v_flag = TRUE;

  printf("Using camera #%d\n",camera_no);
  
  sprintf(buf,"/dev/video%d",camera_no);
  dev_name = calloc(sizeof(char),strlen(buf)+1);
  strcpy(dev_name,buf);
  printf("Camera : %s\n",dev_name);

  open_device();

  query_capabilities();
  query_ctrl();
  list_formats();

  close_device();
  exit(EXIT_SUCCESS);
  return 0;
}
