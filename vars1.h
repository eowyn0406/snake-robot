/**************************************/
/*****���ڶ��������ʹ�õ��ĳ���ֵ*****/
/**************************************/
#ifndef _VARS_H_
#define _VARS_H_

#ifndef BodyX
#define BodyX (0.6)
#endif

#ifndef BodyY
#define BodyY (0.1)
#endif

#ifndef BodyZ
#define BodyZ (0.4)
#endif
//�ߡ�ģ�顱��ز���
#ifndef NumOfModule
#define NumOfModule (7)		//ģ������
#else 
	#error "error in vars.h: The NumOfModule has been defined!!!"
#endif

#ifndef DensityOfModule
#define DensityOfModule (0.08)
#else 
	#error "error in vars.h: The DensityOfModule has been defined!!!"
#endif

#ifndef NumOfHinge
#define NumOfHinge (NumOfModule-1)	//ģ�����HingeJoint���ӣ��ؽ�����
#else 
	#error "error in vars.h: The NumOfHinge has been defined!!!"
#endif



#ifndef ModuleWidth
#define ModuleWidth (0.4)//
#else 
	#error "error in vars.h: The ModuleWidth has been defined!!!"
#endif

#ifndef ModuleLength
#define ModuleLength (0.6)
//(ModuleWidth*sqrt(2))//0.045
#else 
	#error "error in vars.h: The ModuleLength has been defined!!!"
#endif

#ifndef BodyLength
#define BodyLength (0.4)
#endif

#ifndef ModuleHeight
#define ModuleHeight (0.4)//
#else 
	#error "error in vars.h: The ModuleHeight has been defined!!!"
#endif

#ifndef ModuleHeightCen
#define ModuleHeightCen (0.3)//
#else 
	#error "error in vars.h: The ModuleHeight has been defined!!!"
#endif



//�������֡���ز���
#ifndef NumOfWheelPair
#define NumOfWheelPair (NumOfModule)	//ÿ��ģ��1�����ӣ�����������ת���ؽ�����
#else 
	#error "error in vars.h: The NumOfWheelPair has been defined!!!"
#endif

#ifndef DensityOfWheel
#define	DensityOfWheel (0.08)
#else 
	#error "error in vars.h: The DensityOfWheel has been defined!!!"
#endif

#ifndef DensityOfBody
#define	DensityOfBody (0.008)
#else 
	#error "error in vars.h: The DensityOfBody has been defined!!!"
#endif

#ifndef WheelRadius 
#define WheelRadius (0.2)
#else 
	#error "error in vars.h: The WheelRadius has been defined!!!"
#endif

#ifndef WheelWidth
#define WheelWidth (0.1)
#else 
	#error "error in vars.h: The WheelWidth has been defined!!!"
#endif

//���ܱڡ���ز���

#ifndef DensityOfWall
#define DensityOfWall (0.98)
#else 
	#error "error in vars.h: The DensityOfWall has been defined!!!"
#endif

#ifndef WallLength
#define WallLength (0.1)
#else 
	#error "error in vars.h: The WallLength has been defined!!!"
#endif

#ifndef WallWidth
#define WallWidth (0.2)
#else 
	#error "error in vars.h: The WallWidth has been defined!!!"
#endif

#ifndef WallHeight
#define WallHeight (0.1)
#else 
	#error "error in vars.h: The WallHeight has been defined!!!"
#endif

//��������ƫ����
#ifndef deltaY
#define deltaY (0.5)
#else 
	#error "error in vars.h: The deltaY has been defined!!!"
#endif

#ifndef ModuleRadius
#define ModuleRadius (0.2)
#endif

#ifndef NumOfHeadWheels
#define NumOfHeadWheels (1)
#endif

#ifndef WheelLength
#define WheelLength (0.05)
#endif

#endif