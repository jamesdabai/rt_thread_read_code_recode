/*
 * File      : hid.c
 * COPYRIGHT (C) 2008 - 2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Change Logs:
 * Date           Author       Notes
 * 2017-03-13     Urey         the first version
 * 2017-11-16     ZYH          Update to common hid
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtservice.h>
#include <rtdevice.h>

#include "drivers/usb_common.h"
#include "drivers/usb_device.h"

#include "hid.h"

#ifdef RT_USB_DEVICE_HID

struct hid_s
{
    struct rt_device parent;
    struct ufunction *func;
    uep_t ep_in;
    uep_t ep_out;
    int status;
    rt_uint16_t protocol;
    rt_uint8_t report_buf[MAX_REPORT_SIZE];
    struct rt_messagequeue hid_mq;
};

/* CustomHID_ConfigDescriptor */
ALIGN(4)
const rt_uint8_t _report_desc[]=
{
#ifdef RT_USB_DEVICE_HID_KEYBOARD//64个字节
    USAGE_PAGE(1),      0x01,
    USAGE(1),           0x06,
    COLLECTION(1),      0x01,
    REPORT_ID(1),       HID_REPORT_ID_KEYBOARD1,

    USAGE_PAGE(1),      0x07,
    USAGE_MINIMUM(1),   0xE0,
    USAGE_MAXIMUM(1),   0xE7,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_SIZE(1),     0x01,
    REPORT_COUNT(1),    0x08,
    INPUT(1),           0x02,
    REPORT_COUNT(1),    0x01,
    REPORT_SIZE(1),     0x08,
    INPUT(1),           0x01,


    REPORT_COUNT(1),    0x05,
    REPORT_SIZE(1),     0x01,
    USAGE_PAGE(1),      0x08,
    USAGE_MINIMUM(1),   0x01,
    USAGE_MAXIMUM(1),   0x05,
    OUTPUT(1),          0x02,
    REPORT_COUNT(1),    0x01,
    REPORT_SIZE(1),     0x03,
    OUTPUT(1),          0x01,


    REPORT_COUNT(1),    0x06,
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,
    USAGE_PAGE(1),      0x07,
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    INPUT(1),           0x00,
    END_COLLECTION(0),
#if RT_USB_DEVICE_HID_KEYBOARD_NUMBER>1
    /****keyboard2*****/
    USAGE_PAGE(1),      0x01,
    USAGE(1),           0x06,
    COLLECTION(1),      0x01,
    REPORT_ID(1),       HID_REPORT_ID_KEYBOARD2,

    USAGE_PAGE(1),      0x07,
    USAGE_MINIMUM(1),   0xE0,
    USAGE_MAXIMUM(1),   0xE7,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_SIZE(1),     0x01,
    REPORT_COUNT(1),    0x08,
    INPUT(1),           0x02,
    REPORT_COUNT(1),    0x01,
    REPORT_SIZE(1),     0x08,
    INPUT(1),           0x01,

    REPORT_COUNT(1),    0x06,
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,
    USAGE_PAGE(1),      0x07,
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    INPUT(1),           0x00,
    END_COLLECTION(0),
#if RT_USB_DEVICE_HID_KEYBOARD_NUMBER>2
    USAGE_PAGE(1),      0x01,
    USAGE(1),           0x06,
    COLLECTION(1),      0x01,
    REPORT_ID(1),       HID_REPORT_ID_KEYBOARD3,

    USAGE_PAGE(1),      0x07,
    USAGE_MINIMUM(1),   0xE0,
    USAGE_MAXIMUM(1),   0xE7,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_SIZE(1),     0x01,
    REPORT_COUNT(1),    0x08,
    INPUT(1),           0x02,
    REPORT_COUNT(1),    0x01,
    REPORT_SIZE(1),     0x08,
    INPUT(1),           0x01,

    REPORT_COUNT(1),    0x06,
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,
    USAGE_PAGE(1),      0x07,
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    INPUT(1),           0x00,
    END_COLLECTION(0),
#if RT_USB_DEVICE_HID_KEYBOARD_NUMBER>3
    USAGE_PAGE(1),      0x01,
    USAGE(1),           0x06,
    COLLECTION(1),      0x01,
    REPORT_ID(1),       HID_REPORT_ID_KEYBOARD4,

    USAGE_PAGE(1),      0x07,
    USAGE_MINIMUM(1),   0xE0,
    USAGE_MAXIMUM(1),   0xE7,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_SIZE(1),     0x01,
    REPORT_COUNT(1),    0x08,
    INPUT(1),           0x02,
    REPORT_COUNT(1),    0x01,
    REPORT_SIZE(1),     0x08,
    INPUT(1),           0x01,

    REPORT_COUNT(1),    0x06,
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,
    USAGE_PAGE(1),      0x07,
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    INPUT(1),           0x00,
    END_COLLECTION(0),
#endif
#endif
#endif
#endif
    // Media Control
#ifdef RT_USB_DEVICE_HID_MEDIA
    USAGE_PAGE(1),      0x0C,
    USAGE(1),           0x01,
    COLLECTION(1),      0x01,
    REPORT_ID(1),       HID_REPORT_ID_MEDIA,
    USAGE_PAGE(1),      0x0C,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_SIZE(1),     0x01,
    REPORT_COUNT(1),    0x07,
    USAGE(1),           0xB5,             // Next Track
    USAGE(1),           0xB6,             // Previous Track
    USAGE(1),           0xB7,             // Stop
    USAGE(1),           0xCD,             // Play / Pause
    USAGE(1),           0xE2,             // Mute
    USAGE(1),           0xE9,             // Volume Up
    USAGE(1),           0xEA,             // Volume Down
    INPUT(1),           0x02,             // Input (Data, Variable, Absolute)
    REPORT_COUNT(1),    0x01,
    INPUT(1),           0x01,
    END_COLLECTION(0),
#endif

#ifdef RT_USB_DEVICE_HID_GENERAL
    USAGE_PAGE(1),      0x8c,
    USAGE(1),           0x01,
    COLLECTION(1),      0x01,
    REPORT_ID(1),       HID_REPORT_ID_GENERAL,

    REPORT_COUNT(1),    RT_USB_DEVICE_HID_GENERAL_IN_REPORT_LENGTH,
    USAGE(1),           0x03,
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0xFF,
    INPUT(1),           0x02,

    REPORT_COUNT(1),    RT_USB_DEVICE_HID_GENERAL_OUT_REPORT_LENGTH,
    USAGE(1),           0x04,
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0xFF,
    OUTPUT(1),          0x02,
    END_COLLECTION(0),
#endif
#ifdef RT_USB_DEVICE_HID_MOUSE
    USAGE_PAGE(1),      0x01,           // Generic Desktop
    USAGE(1),           0x02,           // Mouse
    COLLECTION(1),      0x01,           // Application
    USAGE(1),           0x01,           // Pointer
    COLLECTION(1),      0x00,           // Physical
    REPORT_ID(1),       HID_REPORT_ID_MOUSE,
    REPORT_COUNT(1),    0x03,
    REPORT_SIZE(1),     0x01,
    USAGE_PAGE(1),      0x09,           // Buttons
    USAGE_MINIMUM(1),   0x1,
    USAGE_MAXIMUM(1),   0x3,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    INPUT(1),           0x02,
    REPORT_COUNT(1),    0x01,
    REPORT_SIZE(1),     0x05,
    INPUT(1),           0x01,
    REPORT_COUNT(1),    0x03,
    REPORT_SIZE(1),     0x08,
    USAGE_PAGE(1),      0x01,
    USAGE(1),           0x30,           // X
    USAGE(1),           0x31,           // Y
    USAGE(1),           0x38,           // scroll
    LOGICAL_MINIMUM(1), 0x81,
    LOGICAL_MAXIMUM(1), 0x7f,
    INPUT(1),           0x06,
    END_COLLECTION(0),
    END_COLLECTION(0),
#endif
}; /* CustomHID_ReportDescriptor */

ALIGN(4)
static struct udevice_descriptor _dev_desc =
{
    USB_DESC_LENGTH_DEVICE,     //bLength;
    USB_DESC_TYPE_DEVICE,       //type;
    USB_BCD_VERSION,            //bcdUSB;
    USB_CLASS_HID,              //bDeviceClass;//如果这里为0，表示该信息在接口描述符中定义的，后面两项都必须为0
    0x00,                       //bDeviceSubClass;
    0x00,                       //bDeviceProtocol;
    64,                         //bMaxPacketSize0;
    _VENDOR_ID,                 //idVendor;
    _PRODUCT_ID,                //idProduct;
    USB_BCD_DEVICE,             //bcdDevice;
    USB_STRING_MANU_INDEX,      //iManufacturer;
    USB_STRING_PRODUCT_INDEX,   //iProduct;
    USB_STRING_SERIAL_INDEX,    //iSerialNumber;
    USB_DYNAMIC,                //bNumConfigurations;
};

//FS and HS needed
ALIGN(4)
static struct usb_qualifier_descriptor dev_qualifier =
{
    sizeof(dev_qualifier),          //bLength
    USB_DESC_TYPE_DEVICEQUALIFIER,  //bDescriptorType
    0x0200,                         //bcdUSB
    USB_CLASS_MASS_STORAGE,         //bDeviceClass
    0x06,                           //bDeviceSubClass
    0x50,                           //bDeviceProtocol
    64,                             //bMaxPacketSize0
    0x01,                           //bNumConfigurations
    0,
};


/* hid interface descriptor */
ALIGN(4)
const static struct uhid_comm_descriptor _hid_comm_desc =
{
#ifdef RT_USB_DEVICE_COMPOSITE
    /* Interface Association Descriptor */
    {
        USB_DESC_LENGTH_IAD,
        USB_DESC_TYPE_IAD,
        USB_DYNAMIC,
        0x01,
        0x03,                       /* bInterfaceClass: HID */
#if defined(RT_USB_DEVICE_HID_KEYBOARD)||defined(RT_USB_DEVICE_HID_MOUSE)
        USB_HID_SUBCLASS_BOOT,    /* bInterfaceSubClass : 1=BOOT, 0=no boot */
#else
        USB_HID_SUBCLASS_NOBOOT,    /* bInterfaceSubClass : 1=BOOT, 0=no boot */
#endif
#if !defined(RT_USB_DEVICE_HID_KEYBOARD)||!defined(RT_USB_DEVICE_HID_MOUSE)||!defined(RT_USB_DEVICE_HID_MEDIA)
        USB_HID_PROTOCOL_NONE,      /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
#elif !defined(RT_USB_DEVICE_HID_MOUSE)
        USB_HID_PROTOCOL_KEYBOARD,  /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
#else
        USB_HID_PROTOCOL_MOUSE,     /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
#endif
        0x00,
    },
#endif
    /* Interface Descriptor */
    {
        USB_DESC_LENGTH_INTERFACE,
        USB_DESC_TYPE_INTERFACE,
        USB_DYNAMIC,                /* bInterfaceNumber: Number of Interface */
        0x00,                       /* bAlternateSetting: Alternate setting 接口编号备用设置*/
        0x02,                       /* bNumEndpoints 该接口有的端点数*/
        //接下来的3个字节，是在设备描述符中没有定义时，这个起作用
        0x03,                       /* bInterfaceClass: HID */
#if defined(RT_USB_DEVICE_HID_KEYBOARD)||defined(RT_USB_DEVICE_HID_MOUSE)
        USB_HID_SUBCLASS_BOOT,    /* bInterfaceSubClass : 1=BOOT, 0=no boot */
#else
        USB_HID_SUBCLASS_NOBOOT,    /* bInterfaceSubClass : 1=BOOT, 0=no boot */
#endif
#if !defined(RT_USB_DEVICE_HID_KEYBOARD)||!defined(RT_USB_DEVICE_HID_MOUSE)||!defined(RT_USB_DEVICE_HID_MEDIA)
        USB_HID_PROTOCOL_NONE,      /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
#elif !defined(RT_USB_DEVICE_HID_MOUSE)
        USB_HID_PROTOCOL_KEYBOARD,  /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
#else
        USB_HID_PROTOCOL_MOUSE,     /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
#endif
        0,                          /* iInterface: Index of string descriptor */
    },

    /* HID Descriptor */
    {
        HID_DESCRIPTOR_SIZE,        /* bLength: HID Descriptor size */
        HID_DESCRIPTOR_TYPE,        /* bDescriptorType: HID */
        0x0110,                     /* bcdHID: HID Class Spec release number */
        0x00,                       /* bCountryCode: Hardware target country */
        0x01,                       /* bNumDescriptors: Number of HID class descriptors to follow */
        {
            {
                0x22,                       /* bDescriptorType */
                sizeof(_report_desc),       /* wItemLength: Total length of Report descriptor */
            },
        },
    },

    /* Endpoint Descriptor IN */
    {
        USB_DESC_LENGTH_ENDPOINT,
        USB_DESC_TYPE_ENDPOINT,
        USB_DYNAMIC | USB_DIR_IN,
        USB_EP_ATTR_INT,
        0x40,
        0x01,
    },

    /* Endpoint Descriptor OUT */
    {
        USB_DESC_LENGTH_ENDPOINT,
        USB_DESC_TYPE_ENDPOINT,
        USB_DYNAMIC | USB_DIR_OUT,
        USB_EP_ATTR_INT,
        0x40,
        0x01,
    },
};

ALIGN(4)
const static char* _ustring[] =
{
    "Language",
    "RT-Thread Team.",
    "RTT HID-Device",
    "32021919830108",
    "Configuration",
    "Interface",
};

static void dump_data(uint8_t *data, rt_size_t size)
{
    rt_size_t i;
    for (i = 0; i < size; i++)
    {
        rt_kprintf("%02x ", *data++);
        if ((i + 1) % 8 == 0)
        {
            rt_kprintf("\n");
        }else if ((i + 1) % 4 == 0){
            rt_kprintf(" ");
        }
    }
}
static void dump_report(struct hid_report * report)
{
    rt_kprintf("\nHID Recived:");
    rt_kprintf("\nReport ID %02x \n", report->report_id);
    dump_data(report->report,report->size);
}

static rt_err_t _ep_out_handler(ufunction_t func, rt_size_t size)
{
    struct hid_s *data;
    struct hid_report report;
    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    data = (struct hid_s *) func->user_data;

    if(size != 0)
    {
        rt_memcpy((void *)&report,(void*)data->ep_out->buffer,size);
        report.size = size-1;
        rt_mq_send(&data->hid_mq,(void *)&report,sizeof(report));
    }

    data->ep_out->request.buffer = data->ep_out->buffer;
    data->ep_out->request.size = EP_MAXPACKET(data->ep_out);
    data->ep_out->request.req_type = UIO_REQUEST_READ_BEST;
    rt_usbd_io_request(func->device, data->ep_out, &data->ep_out->request);
    return RT_EOK;
}

static rt_err_t _ep_in_handler(ufunction_t func, rt_size_t size)
{
    struct hid_s *data;
    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);

    data = (struct hid_s *) func->user_data;
    if(data->parent.tx_complete != RT_NULL)
    {
        data->parent.tx_complete(&data->parent,RT_NULL);
    }
    return RT_EOK;
}

static rt_err_t _hid_set_report_callback(udevice_t device, rt_size_t size)
{
    RT_DEBUG_LOG(RT_DEBUG_USB, ("_hid_set_report_callback\n"));

    if(size != 0)
    {
    }

    dcd_ep0_send_status(device->dcd);

    return RT_EOK;
}

/**
 * This function will handle hid interface bRequest.
 *
 * @param device the usb device object.
 * @param setup the setup bRequest.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _interface_handler(ufunction_t func, ureq_t setup)
{
    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    RT_ASSERT(setup != RT_NULL);

    struct hid_s *data = (struct hid_s *) func->user_data;

    if(setup->wIndex != 0)
        return -RT_EIO;

    switch (setup->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR:
        if((setup->wValue >> 8) == USB_DESC_TYPE_REPORT)
        {
            rt_usbd_ep0_write(func->device, (void *)(&_report_desc[0]), sizeof(_report_desc));
        }
        else if((setup->wValue >> 8) == USB_DESC_TYPE_HID)
        {

            rt_usbd_ep0_write(func->device, (void *)(&_hid_comm_desc.hid_desc), sizeof(struct uhid_descriptor));
        }
        break;
    case USB_HID_REQ_GET_REPORT:
        if(setup->wLength == 0)
        {
            rt_usbd_ep0_set_stall(func->device);
            break;
        }
        if((setup->wLength == 0) || (setup->wLength > MAX_REPORT_SIZE))
            setup->wLength = MAX_REPORT_SIZE;
        rt_usbd_ep0_write(func->device, data->report_buf,setup->wLength);
        break;
    case USB_HID_REQ_GET_IDLE:

        dcd_ep0_send_status(func->device->dcd);
        break;
    case USB_HID_REQ_GET_PROTOCOL:
        rt_usbd_ep0_write(func->device, &data->protocol,2);
        break;
    case USB_HID_REQ_SET_REPORT:

        if((setup->wLength == 0) || (setup->wLength > MAX_REPORT_SIZE))
            rt_usbd_ep0_set_stall(func->device);

        rt_usbd_ep0_read(func->device, data->report_buf, setup->wLength, _hid_set_report_callback);
        break;
    case USB_HID_REQ_SET_IDLE:
        dcd_ep0_send_status(func->device->dcd);
        break;
    case USB_HID_REQ_SET_PROTOCOL:
        data->protocol = setup->wValue;

        dcd_ep0_send_status(func->device->dcd);
        break;
    }

    return RT_EOK;
}


/**
 * This function will run cdc function, it will be called on handle set configuration bRequest.
 *
 * @param func the usb function object.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _function_enable(ufunction_t func)
{
    struct hid_s *data;

    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    data = (struct hid_s *) func->user_data;

    RT_DEBUG_LOG(RT_DEBUG_USB, ("hid function enable\n"));
//
//    _vcom_reset_state(func);
//
    if(data->ep_out->buffer == RT_NULL)
    {
        data->ep_out->buffer        = rt_malloc(HID_RX_BUFSIZE);
    }
    data->ep_out->request.buffer    = data->ep_out->buffer;
    data->ep_out->request.size      = EP_MAXPACKET(data->ep_out);
    data->ep_out->request.req_type  = UIO_REQUEST_READ_BEST;

    rt_usbd_io_request(func->device, data->ep_out, &data->ep_out->request);

    return RT_EOK;
}

/**
 * This function will stop cdc function, it will be called on handle set configuration bRequest.
 *
 * @param func the usb function object.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _function_disable(ufunction_t func)
{
    struct hid_s *data;

    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    data = (struct hid_s *) func->user_data;

    RT_DEBUG_LOG(RT_DEBUG_USB, ("hid function disable\n"));

    if(data->ep_out->buffer != RT_NULL)
    {
        rt_free(data->ep_out->buffer);
        data->ep_out->buffer = RT_NULL;
    }

    return RT_EOK;
}

static struct ufunction_ops ops =
{
    _function_enable,
    _function_disable,
    RT_NULL,
};




/**
 * This function will configure hid descriptor.
 *
 * @param comm the communication interface number.
 * @param data the data interface number.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _hid_descriptor_config(uhid_comm_desc_t hid, rt_uint8_t cintf_nr)
{
#ifdef RT_USB_DEVICE_COMPOSITE
    hid->iad_desc.bFirstInterface = cintf_nr;
#endif

    return RT_EOK;
}
static rt_size_t _hid_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct hid_s *hiddev = (struct hid_s *)dev;
    struct hid_report report;
    if (hiddev->func->device->state == USB_STATE_CONFIGURED)
    {
        report.report_id = pos;
        rt_memcpy((void *)report.report,(void *)buffer,size);
        report.size = size;
        hiddev->ep_in->request.buffer = (void *)&report;
        hiddev->ep_in->request.size = (size+1) > 64 ? 64 : size+1;
        hiddev->ep_in->request.req_type = UIO_REQUEST_WRITE;
        rt_usbd_io_request(hiddev->func->device, hiddev->ep_in, &hiddev->ep_in->request);
        return size;
    }

    return 0;
}
RT_WEAK void HID_Report_Received(hid_report_t report)
{
    dump_report(report);
}
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t hid_thread_stack[512];
static struct rt_thread hid_thread;

static void hid_thread_entry(void* parameter)
{
    struct hid_report report;
    struct hid_s *hiddev;
    hiddev = (struct hid_s *)parameter;
	while(1)
	{
		if(rt_mq_recv(&hiddev->hid_mq, &report, sizeof(report),RT_WAITING_FOREVER) != RT_EOK )
            continue;
		HID_Report_Received(&report);
	}
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops hid_device_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    _hid_write,
    RT_NULL,
};
#endif

static rt_uint8_t hid_mq_pool[(sizeof(struct hid_report)+sizeof(void*))*8];
static void rt_usb_hid_init(struct ufunction *func)
{
    struct hid_s *hiddev;
    hiddev = (struct hid_s *)func->user_data;
    rt_memset(&hiddev->parent, 0, sizeof(hiddev->parent));

#ifdef RT_USING_DEVICE_OPS
    hiddev->parent.ops   = &hid_device_ops;
#else
    hiddev->parent.write = _hid_write;
#endif
    hiddev->func = func;

    rt_device_register(&hiddev->parent, "hidd", RT_DEVICE_FLAG_RDWR);
    rt_mq_init(&hiddev->hid_mq, "hiddmq", hid_mq_pool, sizeof(struct hid_report),
                            sizeof(hid_mq_pool), RT_IPC_FLAG_FIFO);
                            
    rt_thread_init(&hid_thread, "hidd", hid_thread_entry, hiddev,
            hid_thread_stack, sizeof(hid_thread_stack), RT_USBD_THREAD_PRIO, 20);
    rt_thread_startup(&hid_thread);
}


/**
 * This function will create a hid function instance.
 *
 * @param device the usb device object.
 *
 * @return RT_EOK on successful.
 */
ufunction_t rt_usbd_function_hid_create(udevice_t device)
{
    ufunction_t     func;
    struct hid_s   *data;

    uintf_t         hid_intf;
    ualtsetting_t   hid_setting;
    uhid_comm_desc_t hid_desc;

    /* parameter check */
    RT_ASSERT(device != RT_NULL);

    /* set usb device string description */
    rt_usbd_device_set_string(device, _ustring);

    /* create a cdc function */
    func = rt_usbd_function_new(device, &_dev_desc, &ops);
    //not support hs
    //rt_usbd_device_set_qualifier(device, &_dev_qualifier);

    /* allocate memory for cdc vcom data */
    data = (struct hid_s*)rt_malloc(sizeof(struct hid_s));
    rt_memset(data, 0, sizeof(struct hid_s));
    func->user_data = (void*)data;

    /* create an interface object */
    hid_intf = rt_usbd_interface_new(device, _interface_handler);

    /* create an alternate setting object */
    hid_setting = rt_usbd_altsetting_new(sizeof(struct uhid_comm_descriptor));

    /* config desc in alternate setting */
    rt_usbd_altsetting_config_descriptor(hid_setting, &_hid_comm_desc, (rt_off_t)&((uhid_comm_desc_t)0)->intf_desc);

    /* configure the hid interface descriptor */
    _hid_descriptor_config(hid_setting->desc, hid_intf->intf_num);

    /* create endpoint */
    hid_desc = (uhid_comm_desc_t)hid_setting->desc;
    data->ep_out = rt_usbd_endpoint_new(&hid_desc->ep_out_desc, _ep_out_handler);
    data->ep_in  = rt_usbd_endpoint_new(&hid_desc->ep_in_desc, _ep_in_handler);

    /* add the int out and int in endpoint to the alternate setting */
    rt_usbd_altsetting_add_endpoint(hid_setting, data->ep_out);
    rt_usbd_altsetting_add_endpoint(hid_setting, data->ep_in);

    /* add the alternate setting to the interface, then set default setting */
    rt_usbd_interface_add_altsetting(hid_intf, hid_setting);
    rt_usbd_set_altsetting(hid_intf, 0);

    /* add the interface to the mass storage function */
    rt_usbd_function_add_interface(func, hid_intf);

    /* initilize hid */
    rt_usb_hid_init(func);
    return func;
}
struct udclass hid_class = 
{
    .rt_usbd_function_create = rt_usbd_function_hid_create
};

int rt_usbd_hid_class_register(void)
{
    rt_usbd_class_register(&hid_class);
    return 0;
}
INIT_PREV_EXPORT(rt_usbd_hid_class_register);
#endif /* RT_USB_DEVICE_HID */



#if 0//网上关于各种描述符的详细介绍，保留
/* USB设备描述符*/
const uint8_t CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength 描述符的长度*/
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType  描述符的类型（设备描述符为0x01）*/
    0x00,                       /*bcdUSB USB协议的版本*/
    0x02,
        
    0x00,                       /*bDeviceClass 类代码*/
    0x00,                       /*bDeviceSubClass 子类代码*/
    0x00,                       /*bDeviceProtocol  设备所使用的协议*/
    0x40,                       /*bMaxPacketSize 端点0的最大包长*/
        /*idVendor  厂商ID*/
    LOBYTE(USB_VID),                       
    HIBYTE(USB_VID),
        /*idProduct 设备ID*/
    LOBYTE(USB_PID),                       
    HIBYTE(USB_PID),
        
    0x00,                       /*bcdDevice rel 设备版本号*/
    0x02,
         
    1,                          /*描述生产厂家的字符串描述符的索引*/
    2,                          /*描述产品的字符串描述符的索引*/
    3,                          /*产品序列号的字符串描述符的索引*/
    0x01                        /*bNumConfigurations  可能的配置数*/
  }
  ; /* CustomHID_DeviceDescriptor */


/* USB配置描述符 */
/*   All Descriptor s (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
  {
        //以下为配置描述符
    0x09, /* bLength: 端点描述符长度*/
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: 描述符类型 （配置描述符为0x02）  */
        
    LOBYTE(CUSTOMHID_SIZ_CONFIG_DESC),/* wTotalLength: 配置描述符集合总长度 */
    HIBYTE(CUSTOMHID_SIZ_CONFIG_DESC),
        
    0x01,         /* bNumInterfaces: 该配置所支持的接口数*/
    0x01,         /* bConfigurationValue: 该配置的值*/
    0x00,         /* iConfiguration: 描述配置的字符串描述符的索引*/
    0xA0,         /* bmAttributes:该设备的属性（总线供电，支持远程唤醒）
                                        bit 4 ... 0: 保留（必须为0）
                                        bit 5: 1表示支持远程唤醒
                                        bit 6: 1表示设备是自供电 0表示是总线供电
                                        bit 7: 保留（必须为1）    */
        
    0xC8,         /* MaxPower 设备所需要的电流（单位为2mA）400 mA*/
        
        
        //以下为接口描述符
    0x09,         /* bLength: 接口描述符长度*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: 描述符类型 （接口描述符为0x04）*/
    0x00,         /* bInterfaceNumber: 该接口编号（从0开始） */
    0x00,         /* bAlternateSetting: 该接口的备用编号 */
    0x02,         /* bNumEndpoints 该接口所使用的端点数*/
    0x03,         /* bInterfaceClass: 该接口所使用的类*/
    0x00,         /* bInterfaceSubClass : 该接口所使用的子类 */
    0x00,         /* nInterfaceProtocol : 该接口所使用的协议 0 =无，1 =键盘，2 =鼠标*/
    0,            /* iInterface: 描述该接口的字符串描述符的索引 */
        
        //以下为HID描述符
    0x09,         /* bLength: HID描述符长度 */
    HID_DESCRIPTOR_TYPE, /* bDescriptorType: 描述符类型 （接口描述符为0x21） */
    0x10,         /* bcdHID: HID 协议版本号 */
    0x01,
    0x00,         /* bCountryCode: 国家代码 (美式键盘代码为0x21)*/
    0x01,         /* bNumDescriptors:下级描述符的数量*/
    0x22,         /* bDescriptorType 下级描述符的类型*/
    LOBYTE(CUSTOMHID_SIZ_REPORT_DESC),/* wItemLength: 下级描述符的长度*/
    HIBYTE(CUSTOMHID_SIZ_REPORT_DESC),
        
        //以下为端点描述符
    /******************** Descriptor of Custom HID endpoints ******************/
    0x07,          /* bLength:端点描述符长度 */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* 描述符类型 （端点描述符为0x05）*/
    0x81,          /* bEndpointAddress：端点地址
                       bit 3 ... 0：端点号
                       bit 6 ... 4：保留(设置为0)
                       bit 7：0（OUT），1（IN）*/
    0x03,          /* bmAttributes: 端点属性 
                             bit 1 ... 0：表示该端点的传输类型
                                                                            0  控制传输
                                                                            1  等时传输
                                                                            2  批量传输
                                                                            3  中断传输
                                                 bit 7 ... 2：如果该端点是非等时传输 则bit 7 ~ 2 保留(设置为0)
                                                                            如果该端点是等时传输
                                                                            bit 3 ... 2：    表示同步类型
                                                                                                              0 无同步
                                                                                                                1    异步
                                                                                                                2 适配
                                                                                                                3 同步
                                                                            bit 5 ... 4：    表示用途
                                                                                                              0 数据端点
                                                                                                                1    反馈端点
                                                                                                                2 暗含反馈的数据端点
                                                                                                                3 保留
                                                                            bit 7 ... 6：    保留*/
    0x40,          /* wMaxPacketSize: 该端点支持的最大包长度 */
    0x00,
    0x0A,          /* bInterval:端口的查询时间*/
        
        //以下为输出端点1描述符
    0x07,    /*bLength：端点描述符大小 */
    USB_ENDPOINT_DESCRIPTOR_TYPE,    /* 端点描述符类型*/
    0x01,              /* bEndpointAddress：端点地址
                       bit 3 ... 0：端点号
                       bit 6 ... 4：保留
                       bit 7：0（OUT），1（IN）*/
    0x03,    /* bmAttributes: 中断端点 */
    0x40,    /* wMaxPacketSize: 最多64个字节  */
    0x00,
    0x0A,    /* bInterval: 轮询间隔（20毫秒）*/
  };


#endif
