// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  raspai2shougaibutsu.cpp
 * @brief roboto controller
 *
 */
// </rtc-template>

#include "raspai2shougaibutsu.h"

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const raspai2shougaibutsu_spec[] =
#else
static const char* raspai2shougaibutsu_spec[] =
#endif
  {
    "implementation_id", "raspai2shougaibutsu",
    "type_name",         "raspai2shougaibutsu",
    "description",       "roboto controller",
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
raspai2shougaibutsu::raspai2shougaibutsu(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_sensorIn("sensor", m_sensor),
    m_outOut("out", m_out),
    m_endOut("end", m_end)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
raspai2shougaibutsu::~raspai2shougaibutsu()
{
}



RTC::ReturnCode_t raspai2shougaibutsu::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);
  addInPort("sensor", m_sensorIn);
  
  // Set OutPort buffer
  addOutPort("out", m_outOut);
  addOutPort("end", m_endOut);

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t raspai2shougaibutsu::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t raspai2shougaibutsu::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsu::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t raspai2shougaibutsu::onActivated(RTC::UniqueId /*ec_id*/)
{
    //センサー値初期化
    for (int i = 0; i < 4; i++)
    {
        sensor_data[i] = 0;
    }
    return RTC::RTC_OK;
}
RTC::ReturnCode_t raspai2shougaibutsu::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    //ロボットを停止する
    m_out.data.vx = 0;
    m_out.data.va = 0;
    m_outOut.write();
    time = 0;
    jidou = 0;
    senkai = 0;
    chokushin = 1;
    end = 0;
    return RTC::RTC_OK;
}
RTC::ReturnCode_t raspai2shougaibutsu::onExecute(RTC::UniqueId /*ec_id*/)
{
    //入力データの存在確認
    if (m_inIn.isNew())
    {
        //入力データ読み込み
        m_inIn.read();
    }
    if (m_sensorIn.isNew()) {
        m_sensorIn.read();
        for (int i = 0; i < m_sensor.data.length(); i++)
        {
            //入力データ格納
            if (i < 4)
            {
                sensor_data[i] = m_sensor.data[i];
                printf("sensordata[%d]=%d\t", i, sensor_data[i]);
                //センサ値が設定値以上か判定
                if (sensor_data[i] > m_stop_d&&jidou==0)
                {
                    jidou = 1;
                    printf("自動モード起動\n");
                    //センサ値が設定値以上の場合は停止
                }
            }

        }
        printf("\n");
    }
    //前進するときのみ停止するかを判定
    if (jidou == 0) {

        m_out.data.vx = m_in.data.vx;
        m_out.data.va = m_in.data.va;
        m_outOut.write();
    }
    //設定値以上の値のセンサーが無い場合はコンフィギュレーションパラメーターの値で操作
    else if (jidou) {
        printf("chokusen:%d,kaisen:%d,jidou:%d,time:%d\n", chokushin, senkai, jidou,time);
        if (chokushin) {
           
            //センサ値が設定値以上か判定
            if (sensor_data[1] > 300|| sensor_data[2] > 300||sensor_data[0] > 300||sensor_data[3] > 300)
            {
                chokushin = 0;
                senkai = 1;
                printf("旋回\n");
                //センサ値が設定値以上の場合は停止
            }
            
            m_out.data.vx = 0.04;
            m_out.data.va = 0;
            m_outOut.write();
        }
        else if (senkai) {
            if (time > 300) {
                senkai = 0;
                printf("旋回終了\n");
            }
            m_out.data.vx = 0.04;
            m_out.data.va = 1;
            m_outOut.write();
            time++;
        }
        else if (time > 280) {
            if (time > 600) {
                end = 1;
                jidou = 0;
                printf("障害物排除終了\n");
                m_end.data = true;
                m_endOut.write();
            }
            m_out.data.vx = 0.5;
            m_out.data.va = 0;
            m_outOut.write();
            time++;
        }
    }
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t raspai2shougaibutsu::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsu::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsu::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsu::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsu::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void raspai2shougaibutsuInit(RTC::Manager* manager)
  {
    coil::Properties profile(raspai2shougaibutsu_spec);
    manager->registerFactory(profile,
                             RTC::Create<raspai2shougaibutsu>,
                             RTC::Delete<raspai2shougaibutsu>);
  }
  
}
