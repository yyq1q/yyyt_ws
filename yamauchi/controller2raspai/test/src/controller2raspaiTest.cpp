// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  controller2raspaiTest.cpp
 * @brief controller (test code)
 *
 */
// </rtc-template>

#include "controller2raspaiTest.h"

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const controller2raspai_spec[] =
#else
static const char* controller2raspai_spec[] =
#endif
  {
    "implementation_id", "controller2raspaiTest",
    "type_name",         "controller2raspaiTest",
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
controller2raspaiTest::controller2raspaiTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_controllerOut("controller", m_controller),
    m_end1Out("end1", m_end1),
    m_end2Out("end2", m_end2),
    m_ras1In("ras1", m_ras1),
    m_ras2In("ras2", m_ras2)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
controller2raspaiTest::~controller2raspaiTest()
{
}



RTC::ReturnCode_t controller2raspaiTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("ras1", m_ras1In);
  addInPort("ras2", m_ras2In);
  
  // Set OutPort buffer
  addOutPort("controller", m_controllerOut);
  addOutPort("end1", m_end1Out);
  addOutPort("end2", m_end2Out);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t controller2raspaiTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t controller2raspaiTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspaiTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t controller2raspaiTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t controller2raspaiTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t controller2raspaiTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t controller2raspaiTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspaiTest::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspaiTest::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspaiTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t controller2raspaiTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool controller2raspaiTest::runTest()
{
    return true;
}


extern "C"
{
 
  void controller2raspaiTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(controller2raspai_spec);
    manager->registerFactory(profile,
                             RTC::Create<controller2raspaiTest>,
                             RTC::Delete<controller2raspaiTest>);
  }
  
}
