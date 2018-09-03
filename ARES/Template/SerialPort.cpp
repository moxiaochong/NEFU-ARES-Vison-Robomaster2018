//////////////////////////////////////////////////////////////////////////  
/// COPYRIGHT NOTICE  
/// Copyright (c) 2009, 华中科技大学tickTick Group  （版权声明）  
/// All rights reserved.  
///   
/// @file    SerialPort.cpp    
/// @brief   串口通信类的实现文件  
///  
/// 本文件为串口通信类的实现代码  
///  
/// @version 1.0     
/// @author  卢俊    
/// @E-mail：lujun.hust@gmail.com  
/// @date    2010/03/19  
///   这个使用的是原版的初始化；
///  
///  修订说明：  
//////////////////////////////////////////////////////////////////////////  

#include "Stdafx.h"  
#include "SerialPort.h"  
#include <process.h>  
#include <iostream>  
//外部声明
//extern char recieve_data[COM_BUF_LEN];
//extern char send_data[COM_BUF_LEN];

/** 线程退出标志 */
bool CSerialPort::s_bExit = false;
/** 当串口无数据时,sleep至下次查询间隔的时间,单位:毫秒 */
const UINT SLEEP_TIME_INTERVAL = 3;

CSerialPort::CSerialPort(void)
	: m_hListenThread(INVALID_HANDLE_VALUE)
{
	m_hComm = INVALID_HANDLE_VALUE;
	m_hListenThread = INVALID_HANDLE_VALUE;

	InitializeCriticalSection(&m_csCommunicationSync);

}

CSerialPort::~CSerialPort(void)
{
	CloseListenTread();
	ClosePort();
	DeleteCriticalSection(&m_csCommunicationSync);
}

//串口初始化（打开并设置串口）
//bool CSerialPort::InitPort(UINT portNo)
//{
//	/** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */
//	if (!openPort(portNo))
//	{
//		return false;
//	}
//	/** 进入临界段 */
//	EnterCriticalSection(&m_csCommunicationSync);
//
//	//设置串口缓冲区大小,要注意设置稍大一些,避免缓冲区溢出
//	if (!SetupComm(m_hComm, 1024, 1024))
//	{
//		printf("设置缓冲区失败！\n");
//		ClosePort();
//		return false;
//	}
//	/** 设置串口的超时时间,均设为0,表示不使用超时限制 */
//	COMMTIMEOUTS  CommTimeouts;
//	CommTimeouts.ReadIntervalTimeout = 0;
//	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
//	CommTimeouts.ReadTotalTimeoutConstant = 0;
//	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
//	CommTimeouts.WriteTotalTimeoutConstant = 0;
//	if (!SetCommTimeouts(m_hComm, &CommTimeouts))
//	{
//		printf("设置超时失败！\n");
//		ClosePort();
//		return false;
//	}
//	//设置串口参数
//	DCB dcb = { 0 };
//	if (!GetCommState(m_hComm, &dcb))
//	{
//		printf("GetCommState fail\n");
//		ClosePort();
//		return false;
//	}
//	dcb.DCBlength = sizeof(dcb);
//	if (!BuildCommDCB("115200,n,8,1", &dcb))//填充ＤＣＢ的数据传输率、奇偶校验类型、数据位、停止位
//	{
//		printf("BuileCOmmDCB fail\n");
//		ClosePort();
//		return false;
//	}
//	if (!SetCommState(m_hComm, &dcb))
//	{
//		printf("SetCommState fail!\n");
//		ClosePort();
//		return false;
//	}
//	/**  清空串口缓冲区 */
//	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
//	/** 离开临界段 */
//	LeaveCriticalSection(&m_csCommunicationSync);
//
//	return TRUE;
//}

bool CSerialPort::InitPort(UINT portNo /*= 1*/, UINT baud /*= CBR_9600*/, char parity /*= 'N'*/,
	UINT databits /*= 8*/, UINT stopsbits /*= 1*/, DWORD dwCommEvents /*= EV_RXCHAR*/)
{

	/** 临时变量,将制定参数转化为字符串形式,以构造DCB结构 */
	char szDCBparam[50];
	sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);

	/** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */
	if (!openPort(portNo))
	{
		return false;
	}

	/** 进入临界段 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 是否有错误发生 */
	BOOL bIsSuccess = TRUE;

	/** 在此可以设置输入输出的缓冲区大小,如果不设置,则系统会设置默认值.
	*  自己设置缓冲区大小时,要注意设置稍大一些,避免缓冲区溢出
	*/
	/*if (bIsSuccess )
	{
	bIsSuccess = SetupComm(m_hComm,10,10);
	}*/

	/** 设置串口的超时时间,均设为0,表示不使用超时限制 */
	COMMTIMEOUTS  CommTimeouts;
	CommTimeouts.ReadIntervalTimeout = 0;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 0;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant = 0;
	if (bIsSuccess)
	{
		bIsSuccess = SetCommTimeouts(m_hComm, &CommTimeouts);
	}

	DCB  dcb;
	if (bIsSuccess)
	{
		// 将ANSI字符串转换为UNICODE字符串    
		DWORD dwNum = MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, NULL, 0);
		wchar_t *pwText = new wchar_t[dwNum];
		if (!MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, pwText, dwNum))
		{
			bIsSuccess = TRUE;
		}

		/** 获取当前串口配置参数,并且构造串口DCB参数 */
		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB(pwText, &dcb);
		/** 开启RTS flow控制 */
		dcb.fRtsControl = RTS_CONTROL_ENABLE;

		/** 释放内存空间 */
		delete[] pwText;
	}

	if (bIsSuccess)
	{
		/** 使用DCB参数配置串口状态 */
		bIsSuccess = SetCommState(m_hComm, &dcb);
	}

	/**  清空串口缓冲区 */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** 离开临界段 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return bIsSuccess == TRUE;
}

//串口初始化函数
void CSerialPort::my_InitPort(UINT portNo)
{
	if (InitPort(portNo))
	{
		printf("串口初始化成功！\n");
	}
	else
	{
		printf("串口初始化失败！\n");
	}
}

//设置串口参数
bool CSerialPort::InitPort(UINT portNo, const LPDCB& plDCB)
{
	/** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */
	if (!openPort(portNo))
	{
		return false;
	}
	/** 进入临界段 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 配置串口参数 */
	if (!SetCommState(m_hComm, plDCB))
	{
		return false;
	}
	/**  清空串口缓冲区 */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** 离开临界段 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

//关闭串口
void CSerialPort::ClosePort()
{
	/** 如果有串口被打开，关闭它 */
	if (m_hComm != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hComm);
		m_hComm = INVALID_HANDLE_VALUE;
	}
}

//关闭串口
void CSerialPort::my_Close_Port()
{
	ClosePort();
}

//打开串口
bool CSerialPort::openPort(UINT portNo)
{
	/** 进入临界段 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 把串口的编号转换为设备名 */
	char szPort[50];
	sprintf_s(szPort, "COM%d", portNo);

	/** 打开指定的串口 */
	m_hComm = CreateFileA(szPort,  /** 设备名,COM1,COM2等 */
		GENERIC_READ | GENERIC_WRITE, /** 访问模式,可同时读写 */
		0,                            /** 共享模式,0表示不共享 */
		NULL,                         /** 安全性设置,一般使用NULL */
		OPEN_EXISTING,                /** 该参数表示设备必须存在,否则创建失败 */
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
		0);

	/** 如果打开失败，释放资源并返回 */
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}

	/** 退出临界区 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

bool CSerialPort::OpenListenThread()
{
	/** 检测线程是否已经开启了 */
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** 线程已经开启 */
		return false;
	}

	s_bExit = false;
	/** 线程ID */
	UINT threadId;
	/** 开启串口数据监听线程 */
	m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);
	if (!m_hListenThread)
	{
		return false;
	}
	/** 设置线程的优先级,高于普通线程 */
	if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_ABOVE_NORMAL))
	{
		return false;
	}

	return true;
}

void CSerialPort::my_OpenListenThread()
{
	if (OpenListenThread())
	{
		printf("开始监听串口！\n");
	}
	else
	{
		printf("尝试监听串口失败 ...\n");
	}
}

bool CSerialPort::CloseListenTread()
{
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** 通知线程退出 */
		s_bExit = true;

		/** 等待线程退出 */
		Sleep(10);

		/** 置线程句柄无效 */
		CloseHandle(m_hListenThread);
		m_hListenThread = INVALID_HANDLE_VALUE;
	}
	return true;
}

DWORD CSerialPort::GetBytesInCOM()
{
	DWORD dwError = 0;  /** 错误码 */
	COMSTAT  comstat;   /** COMSTAT结构体,记录通信设备的状态信息 */
	memset(&comstat, 0, sizeof(COMSTAT));

	UINT BytesInQue = 0;
	/** 在调用ReadFile和WriteFile之前,通过本函数清除以前遗留的错误标志 */
	if (ClearCommError(m_hComm, &dwError, &comstat))
	{
		BytesInQue = comstat.cbInQue; /** 获取在输入缓冲区中的字节数 */
	}

	return BytesInQue;
}

UINT WINAPI CSerialPort::ListenThread(void* pParam)
{

	/** 得到本类的指针 */
	CSerialPort *pSerialPort = reinterpret_cast<CSerialPort*>(pParam);

	// 线程循环,轮询方式读取串口数据    
	while (!pSerialPort->s_bExit)
	{
		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		/** 如果串口输入缓冲区中无数据,则休息一会再查询 */
		if (BytesInQue == 0)
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}

		/** 读取输入缓冲区中的数据并输出显示 */
		/*char cRecved = 0x00;
		do
		{
		cRecved = 0x00;
		if (pSerialPort->ReadChar(cRecved) == true)
		{
		std::cout << cRecved;
		continue;
		}
		} while (--BytesInQue);*/
		DWORD RecieveLen = pSerialPort->ReadBlock(pSerialPort->my_RecieveBuff, BytesInQue);
		if (RecieveLen != 0)
		{
			if (pSerialPort->MyRecieveData(pSerialPort->my_RecieveBuff, pSerialPort->my_recieve_data))
			{
				//printf("=========数据校验成功！\n");
				//printf("\n");
				/*char cmd = recieve_data[1];
				int a = *(INT16*)&recieve_data[4];
				int b = *(INT16*)&recieve_data[6];
				int c = *(INT16*)&recieve_data[8];
				printf("成功接收===%c===%d===%d===%d \n", cmd, a, b, c);*/
			}
			else
			{
				//printf("=========数据校验失败！\n");
				//printf("\n");


			}
		}

	}

	return 0;
}

DWORD CSerialPort::ReadBlock(char* R_buff, DWORD len)
{
	BOOL  bResult;
	DWORD ReadBytesWant = COM_BUF_LEN;
	DWORD ReadBytesReal = 0;
	OVERLAPPED MyOsRead;
	memset(&MyOsRead, 0, sizeof(OVERLAPPED));
	MyOsRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	ReadBytesWant = min(ReadBytesWant, len);
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	/** 临界区保护 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 从缓冲区读取数据 */
	bResult = ReadFile(m_hComm, R_buff, ReadBytesWant, &ReadBytesReal, &MyOsRead);
	if ((!bResult))
	{
		if (GetLastError() == ERROR_IO_PENDING)
		{
			GetOverlappedResult(m_hComm,
				&MyOsRead, &ReadBytesReal, TRUE);
			// GetOverlappedResult函数的最后一个参数设为TRUE，
			//函数会一直等待，直到读操作完成或由于错误而返回。
			/** 清空串口缓冲区 */
			PurgeComm(m_hComm, PURGE_RXABORT | PURGE_RXCLEAR);
			LeaveCriticalSection(&m_csCommunicationSync);
			return ReadBytesReal;
		}
		/** 清空串口缓冲区 */
		PurgeComm(m_hComm, PURGE_RXABORT | PURGE_RXCLEAR);
		LeaveCriticalSection(&m_csCommunicationSync);
		return 0;
	}
	/** 清空串口缓冲区 */
	PurgeComm(m_hComm, PURGE_RXABORT | PURGE_RXCLEAR);
	LeaveCriticalSection(&m_csCommunicationSync);
	return ReadBytesReal;
}

bool CSerialPort::MyRecieveData(char* srcBuff, char* dstBuff)
{
	bool flag = true;
	/** 临界区保护 */
	EnterCriticalSection(&m_csCommunicationSync);
	for (int i = 0; i < DATA_LEN; i++)
	{
		dstBuff[i] = srcBuff[i];
	}
	if (UART_Receive_Buff(srcBuff))//CRC校验
	{
		flag = true;
	}
	else
	{
		flag = false;
	}
	//离开临界保护区
	LeaveCriticalSection(&m_csCommunicationSync);
	return flag;
}

bool CSerialPort::WriteData(char* pData, UINT16 length)
{
	BOOL   bResult;
	DWORD dwError;
	DWORD  WriteBytesReal;
	OVERLAPPED MyOsWrite;
	memset(&MyOsWrite, 0, sizeof(OVERLAPPED));
	MyOsWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	/** 临界区保护 */
	EnterCriticalSection(&m_csCommunicationSync);
	if (ClearCommError(m_hComm, &dwError, NULL))
	{
		PurgeComm(m_hComm, PURGE_TXABORT | PURGE_TXCLEAR);
	}
	else
	{
		return false;
	}
	if (!WriteFile(m_hComm, pData, length, &WriteBytesReal, &MyOsWrite))
	{
		if (GetLastError() == ERROR_IO_PENDING)
		{
			while (!GetOverlappedResult(m_hComm, &MyOsWrite, &WriteBytesReal, FALSE))
				//成功返回非0，失败返回0
			{
				if (GetLastError() == ERROR_IO_INCOMPLETE)
				{
					continue;
				}
				else
				{
					ClearCommError(m_hComm, &dwError, NULL);
					/** 离开临界区 */
					LeaveCriticalSection(&m_csCommunicationSync);
					return false;
				}
			}
		}
	}
	/** 离开临界区 */
	LeaveCriticalSection(&m_csCommunicationSync);
	return true;
}



//=======================================================//
//CRC16打包
void CSerialPort::ISO14443AAppendCRCA(void* Buffer, UINT16 ByteCount)
{
	UINT16 Checksum = 0x6363;
	BYTE* DataPtr = (BYTE*)Buffer;

	while (ByteCount--) {
		BYTE Byte = *DataPtr++;

		Byte ^= (BYTE)(Checksum & 0x00FF);
		Byte ^= Byte << 4;

		Checksum = (Checksum >> 8) ^ ((UINT16)Byte << 8) ^
			((UINT16)Byte << 3) ^ ((UINT16)Byte >> 4);
	}

	*DataPtr++ = (Checksum >> 0) & 0x00FF;
	*DataPtr = (Checksum >> 8) & 0x00FF;
}

//CRC16解包
BYTE CSerialPort::ISO14443ACheckCRCA(void* Buffer, UINT16 ByteCount)
{
	UINT16 Checksum = 0x6363;
	BYTE* DataPtr = (BYTE*)Buffer;

	while (ByteCount--) {
		BYTE Byte = *DataPtr++;

		Byte ^= (BYTE)(Checksum & 0x00FF);
		Byte ^= Byte << 4;

		Checksum = (Checksum >> 8) ^ ((UINT16)Byte << 8) ^
			((UINT16)Byte << 3) ^ ((UINT16)Byte >> 4);
	}

	return (DataPtr[0] == ((Checksum >> 0) & 0xFF)) && (DataPtr[1] == ((Checksum >> 8) & 0xFF));
}

//检查数据长度是否合格
BYTE CSerialPort::ISO14443ACheckLen(BYTE* Buffer)
{
	if ((Buffer[0] + Buffer[1]) == 0xff && Buffer[0]<COMMAND_BUF_LEN - 2)
		return 1;
	else
		return 0;
}

//数据包发送
bool CSerialPort::UART_Send_Buff(char command, char *data_input, UINT16 data_len)
{
	BYTE  Buffer[COMMAND_BUF_LEN];
	Buffer[0] = 0x55;//帧头
	Buffer[1] = command;//命令 
	Buffer[2] = data_len;//数据长度
	Buffer[3] = 0xff - data_len;//取反
	memcpy(Buffer + HEAD_LEN, data_input, data_len);//拷贝数据
	ISO14443AAppendCRCA(Buffer, data_len + HEAD_LEN);//打包CRC
	if (WriteData((char*)Buffer, data_len + HEAD_LEN + 2))
		return true;
	else
	{
		return false;
	}
}

//数据接收
bool CSerialPort::UART_Receive_Buff(char* arrRC_Buf)
{
	if (arrRC_Buf[0] == 0x55)
	{
		if (ISO14443ACheckLen((BYTE*)(arrRC_Buf + HEAD_LEN - 2)))
		{
			if (ISO14443ACheckCRCA(arrRC_Buf, (UINT16)(arrRC_Buf[2] + HEAD_LEN)))
			{

			}
			else
			{
				return FALSE;
			}
		}
		else
		{
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}
	return TRUE;
}

bool CSerialPort::ReadChar(char &cRecved)
{
	BOOL  bResult = TRUE;
	DWORD BytesRead = 0;
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	/** 临界区保护 */
	EnterCriticalSection(&m_csCommunicationSync);

	/** 从缓冲区读取一个字节的数据 */
	bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);
	if ((!bResult))
	{
		/** 获取错误码,可以根据该错误码查出错误原因 */
		DWORD dwError = GetLastError();

		/** 清空串口缓冲区 */
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** 离开临界区 */
	LeaveCriticalSection(&m_csCommunicationSync);

	return (BytesRead == 1);

}