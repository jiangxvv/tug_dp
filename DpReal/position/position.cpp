#include "position.h"
#include <array>
#include <QDebug>
#include <conio.h>

#ifdef _WIN32
#define sleep Sleep
#else
#include <unistd.h>
#endif

using namespace std;

Position::Position()
{
    //const char Title = "连接6自由度摄像机时发生错误";
    const char serverAddr[] = "192.168.253.1";
    const unsigned short basePort = 22222;
//  const char * serverAddr = serverAddr_value;
//  const unsigned short basePort = basePort_value;
    const int majorVersion = 1;
    const int minorVersion = 13;
    const bool bigEndian = false;

    bool dataAvailable = false;
    bool streamFrames = false;
    unsigned short udpPort = 6734;
//    if (!rtProtocol.Connected())
//    {
//        if (!rtProtocol.Connect(serverAddr, basePort, &udpPort, majorVersion, minorVersion, bigEndian))
//        {
//            printf("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
//            sleep(1);
//            //continue;
//        }
//    }

//    if (!dataAvailable)
//    {
//        if (!rtProtocol.Read6DOFSettings(dataAvailable))
//        {
//            printf("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
//            sleep(1);
//            //continue;
//        }
//    }

//    if (!streamFrames)
//    {
//        if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d))
//        {
//            printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
//            sleep(1);
//            //continue;
//        }
//        streamFrames = true;

//        printf("Starting to streaming 6DOF data\n\n");
//    }
    rtProtocol.Connect(serverAddr, basePort, 0, majorVersion, minorVersion, bigEndian);
    //bool dataAvailable;
    rtProtocol.Read6DOFSettings(dataAvailable);
    rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, 0, NULL, CRTProtocol::cComponent6dEuler);

}

Position::~Position()
{
    rtProtocol.StopCapture();
    rtProtocol.Disconnect();
}

bool Position::GetPosition()
{
//    bool abort = false;

//    while (!abort)
//    {
        CRTPacket::EPacketType packetType;

        if (rtProtocol.ReceiveRTPacket(packetType, true) > 0)
        {
            if (packetType == CRTPacket::PacketData)
            {
                float fX, fY, fZ;
                float fAng1, fAng2, fAng3;

                CRTPacket* rtPacket = rtProtocol.GetRTPacket();

                printf("Timestamp %10I64d \tFrame %d\n", rtPacket->GetTimeStamp(), rtPacket->GetFrameNumber());
                printf("======================================================================================================================\n");


                for (unsigned int i = 0; i < rtPacket->Get6DOFEulerBodyCount(); i++)
                {
                    if (rtPacket->Get6DOFEulerBody(i, fX, fY, fZ, fAng1, fAng2, fAng3))
                    {
                        const char* pTmpStr = rtProtocol.Get6DOFBodyName(i);
                        if (pTmpStr)
                        {
                            printf("%-12s ", pTmpStr);
                        }
                        else
                        {
                            printf("Unknown     ");
                        }
                        printf("Pos: %9.3f %9.3f %9.3f    Rot: %6.3f %6.3f %6.3f\n",
                            fX, fY, fZ, fAng1, fAng2, fAng3);
                                            //传输数据
                                            position_6DOF_.at(0) = fX / 1000.0;
                                            position_6DOF_.at(1) = fY / 1000.0;
                                            position_6DOF_.at(2) = fZ / 1000.0;
                                            position_6DOF_.at(3) = fAng1;
                                            position_6DOF_.at(4) = fAng2;
                                            position_6DOF_.at(5) = fAng3 / 180.0 * 3.14159;
//                                            qDebug() << "position_6DOF_.at(0): " << position_6DOF_.at(0);
//                                            qDebug() << "rtPacket->GetTimeStamp(): " << rtPacket->GetTimeStamp();
                                                               }
                }
                printf("\n");
                //delete rtPacket;
            }
//        }

//        if (abort = (_kbhit() != 0))
//        {
//            _getch(); // Consume key pressed
//        }
    }
//    }
//    printf("ok");
//    // static int cnt = 10; // 不清楚这个处理的作用
//    CRTPacket::EPacketType packetType;

//    if (rtProtocol.ReceiveRTPacket(packetType, true) > 0)
//    {
//        if (packetType == CRTPacket::PacketData)
//        {
//            float fX, fY, fZ;
//            float fAng1, fAng2, fAng3;
//            CRTPacket* rtPacket = rtProtocol.GetRTPacket();

//            printf("Frame %d\n", rtPacket->GetFrameNumber());
//            printf("======================================================================================================================\n");

//            for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
//            {
//                if (rtPacket->Get6DOFEulerBody(i, fX, fY, fZ, fAng1, fAng2, fAng3))
//                {
//                    const char* pTmpStr = rtProtocol.Get6DOFBodyName(i);
//                    if (pTmpStr)
//                    {
//                        printf("%-12s ", pTmpStr);
//                    }
//                    else
//                    {
//                        printf("Unknown     ");
//                    }
//                    printf("Pos: %9.3f %9.3f %9.3f    Rot: %6.3f %6.3f %6.3f\n",
//                        fX, fY, fZ, fAng1, fAng2, fAng3);

//                    qDebug() << fX, fY, fZ, fAng1, fAng2, fAng3;
//                    //传输数据
//                    position_6DOF_.at(0) = fX;
//                    position_6DOF_.at(1) = fY;
//                    position_6DOF_.at(2) = fZ;
//                    position_6DOF_.at(3) = fAng1;
//                    position_6DOF_.at(4) = fAng2;
//                    position_6DOF_.at(5) = fAng3;
//                }
//            }
//            printf("\n");
//        }
//    }

}

std::array<double, 6> Position::PositionOutput()
{
    for(int i = 0; i < 6; i++)
    {
        position_output_.at(i) = position_6DOF_.at(i) - position_zero_.at(i) + virtual_postion_.at(i);
    }
//    qDebug() << "position_output_.at(0): " << position_output_.at(0);
//    qDebug() << "position_zero_.at(0): " << position_zero_.at(0);
    return position_output_;

}
