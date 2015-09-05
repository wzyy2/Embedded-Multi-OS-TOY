#include <stdlib.h>
#include <rtthread.h>
#include <components.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "hsi2c.h"
#include <vmm.h>

static unsigned long I2C0_BASE;

static void SetupI2C(void)
{
    I2C0_BASE = vmm_find_iomap("SOC_I2C_0_REGS");

    /* Enable the clock for I2C0 */
    I2C0ModuleClkConfig();

    I2CPinMuxSetup(0);
    /* Put i2c in reset/disabled state */
    I2CMasterDisable(I2C0_BASE);
    /* Disable auto Idle functionality */
    I2CAutoIdleDisable(I2C0_BASE);
    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(I2C0_BASE, 48000000, 12000000, 100000);
    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(I2C0_BASE, I2C_SLAVE_ADDR);
    /* Bring I2C out of reset */
    I2CMasterEnable(I2C0_BASE);
}

/*
** I2C Interrupt Service Routine. This function will read and write
** data through I2C bus. 
*/
static void mpu6050_isr(int irqno, void* param)
{
    unsigned int status = 0;

    /* Get only Enabled interrupt status */
    status = I2CMasterIntStatus(I2C0_BASE);

    /* 
    ** Clear all enabled interrupt status except receive ready and
    ** transmit ready interrupt status 
    */
    I2CMasterIntClearEx(I2C0_BASE,
                        (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));
                        
    if(status & I2C_INT_RECV_READY)
    {
         /* Receive data from data receive register */
        dataFromSlave[rCount++] = I2CMasterDataGet(I2C0_BASE);

    /* Clear receive ready interrupt status */  
        I2CMasterIntClearEx(I2C0_BASE,  I2C_INT_RECV_READY);
        
         if(rCount == numOfBytes)
         {
              /* Disable the receive ready interrupt */
              I2CMasterIntDisableEx(I2C0_BASE, I2C_INT_RECV_READY);
              /* Generate a STOP */
              I2CMasterStop(I2C0_BASE);
              
         }
    }

    if (status & I2C_INT_TRANSMIT_READY)
    {
        /* Put data to data transmit register of i2c */
        I2CMasterDataPut(I2C0_BASE, dataToSlave[tCount++]);

        /* Clear Transmit interrupt status */
    I2CMasterIntClearEx(I2C0_BASE, I2C_INT_TRANSMIT_READY);         
                        
         if(tCount == numOfBytes)
         {
              /* Disable the transmit ready interrupt */
              I2CMasterIntDisableEx(I2C0_BASE, I2C_INT_TRANSMIT_READY);
         }

    }
        
    if (status & I2C_INT_STOP_CONDITION)
    {
         /* Disable transmit data ready and receive data read interupt */
         I2CMasterIntDisableEx(I2C0_BASE, I2C_INT_TRANSMIT_READY |
                                               I2C_INT_RECV_READY     |
                           I2C_INT_STOP_CONDITION);
         flag = 0;
    }
   
    if(status & I2C_INT_NO_ACK)
    {
         I2CMasterIntDisableEx(I2C0_BASE, I2C_INT_TRANSMIT_READY  |
                                               I2C_INT_RECV_READY      |
                                               I2C_INT_NO_ACK          |
                                               I2C_INT_STOP_CONDITION);
         /* Generate a STOP */
         I2CMasterStop(I2C0_BASE);

         flag = 0;
    }
}

int mpu6050_init()
{
    /*
    ** Configures I2C to Master mode to generate start
    ** condition on I2C bus and to transmit data at a
    ** speed of 100khz
    */
    SetupI2C();

    rt_hw_interrupt_install(SYS_INT_I2C0INT, mpu6050_isr, NULL, "mpu6050");
    rt_hw_interrupt_mask(SYS_INT_I2C0INT);

    int result = mpu_init();

    if(!result)
    {            
        rt_kprintf("mpu initialization complete......\n ");        //mpu_set_sensor
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            rt_kprintf("mpu_set_sensor complete ......\n");
        else
            rt_kprintf("mpu_set_sensor come across error ......\n");

        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))         //mpu_configure_fifo
            rt_kprintf("mpu_configure_fifo complete ......\n");
        else
            rt_kprintf("mpu_configure_fifo come across error ......\n");
        
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))          //mpu_set_sample_rate
            rt_kprintf("mpu_set_sample_rate complete ......\n");
        else
            rt_kprintf("mpu_set_sample_rate error ......\n");
        
        if(!dmp_load_motion_driver_firmware())        //dmp_load_motion_driver_firmvare
            rt_kprintf("dmp_load_motion_driver_firmware complete ......\n");
        else
            rt_kprintf("dmp_load_motion_driver_firmware come across error ......\n");
        
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))      //dmp_set_orientation
            rt_kprintf("dmp_set_orientation complete ......\n");
        else
            rt_kprintf("dmp_set_orientation come across error ......\n");
        
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))            //dmp_enable_feature
            rt_kprintf("dmp_enable_feature complete ......\n");
        else
            rt_kprintf("dmp_enable_feature come across error ......\n");
        
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))        //dmp_set_fifo_rate
            rt_kprintf("dmp_set_fifo_rate complete ......\n");
        else
            rt_kprintf("dmp_set_fifo_rate come across error ......\n");
        
        run_self_test();
        
        if(!mpu_set_dmp_state(1))
            rt_kprintf("mpu_set_dmp_state complete ......\n");
        else
            rt_kprintf("mpu_set_dmp_state come across error ......\n");
    }
    while(1) 
    {
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
            &more);  
        /* Gyro and accel data are written to the FIFO by the DMP in chip
        * frame and hardware units. This behavior is convenient because it
        * keeps the gyro and accel outputs of dmp_read_fifo and
        * mpu_read_fifo consistent.
        */
    /*  if (sensors & INV_XYZ_GYRO )
            send_packet(PACKET_TYPE_GYRO, gyro);
        if (sensors & INV_XYZ_ACCEL)
            send_packet(PACKET_TYPE_ACCEL, accel);      */
        /* Unlike gyro and accel, quaternions are written to the FIFO in
        * the body frame, q30. The orientation is set by the scalar passed
        * to dmp_set_orientation during initialization.
        */
        if (sensors & INV_WXYZ_QUAT )
        {
            q0=quat[0] / q30;   
            q1=quat[1] / q30;
            q2=quat[2] / q30;
            q3=quat[3] / q30;
            Pitch  = asin(2 * q1 * q3 - 2 * q0* q2)* 57.3; // pitch
            Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
            Yaw =   atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;

        //  printf("Pitch=%.2f ,Roll=%.2f ,Yaw=%.2f \n",Pitch,Roll,Yaw);    
            UART1_ReportIMU(Yaw*10, Pitch*10, Roll*10,0,0,0,100);
            delay_ms(10);
        }

    }


    return 0;
}
