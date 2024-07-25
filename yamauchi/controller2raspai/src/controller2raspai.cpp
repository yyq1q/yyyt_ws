// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  controller2raspai.cpp
 * @brief controller
 *
 */
// </rtc-template>

#include "controller2raspai.h"

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const controller2raspai_spec[] =
#else
static const char* controller2raspai_spec[] =
#endif
  {
    "implementation_id", "controller2raspai",
    "type_name",         "controller2raspai",
    "description",       "controller",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
controller2raspai::controller2raspai(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_controllerIn("controller", m_controller),
    m_end1In("end1", m_end1),
    m_end2In("end2", m_end2),
    m_ras1Out("ras1", m_ras1),
    m_ras2Out("ras2", m_ras2)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
controller2raspai::~controller2raspai()
{
}



RTC::ReturnCode_t controller2raspai::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("controller", m_controllerIn);
  addInPort("end1", m_end1In);
  addInPort("end2", m_end2In);
  
  // Set OutPort buffer
  addOutPort("ras1", m_ras1Out);
  addOutPort("ras2", m_ras2Out);

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t controller2raspai::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t controller2raspai::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspai::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}
RTC::ReturnCode_t controller2raspai::onActivated(RTC::UniqueId /*ec_id*/)
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t controller2raspai::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    m_ras1.data.vx = 0;
    m_ras1.data.va = 0;
    m_ras2.data.vx = 0;
    m_ras2.data.va = 0;
    m_ras1Out.write();
    m_ras2Out.write();
    return RTC::RTC_OK;
}

RTC::ReturnCode_t controller2raspai::onExecute(RTC::UniqueId /*ec_id*/)
{
    if (m_end1In.isNew()) {
        if (m_end1.data)Activate_robot = 2;
        printf("////////////END1////////////////////////////////////////////");
    }
    if (m_end2In.isNew()) {
        if (m_end2.data)Activate_robot = 0;
        printf("////////////END2////////////////////////////////////////////");
    }
    if (m_controllerIn.isNew())
    {
        m_controllerIn.read();
        if (Activate_robot == 0)Activate_robot = 1;
        if (Activate_robot == 1) {
            m_ras1.data.vx = m_controller.data.vx;
            m_ras1.data.va = m_controller.data.va;
            printf("robot=%d,vx = %lf,va=%lf\n", Activate_robot, m_ras1.data.vx, m_ras1.data.vx);
            m_ras1Out.write();
        }
        else if (Activate_robot == 2) {
            m_ras2.data.vx = m_controller.data.vx;
            m_ras2.data.va = m_controller.data.va;
            printf("robot=%d,vx = %lf,va=%lf\n", Activate_robot, m_ras2.data.vx, m_ras2.data.vx);
            m_ras2Out.write();
        }
    }
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t controller2raspai::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspai::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspai::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspai::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspai::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void controller2raspaiInit(RTC::Manager* manager)
  {
    coil::Properties profile(controller2raspai_spec);
    manager->registerFactory(profile,
                             RTC::Create<controller2raspai>,
                             RTC::Delete<controller2raspai>);
  }
  
}
