/**************************************/
/*****用于定义程序中使用到的常量值*****/
/**************************************/
#ifndef _VARS_H_
#define _VARS_H_
#define PassWheelDensity (0.0001)
#define PassWheelRadius (0.2)
#define deltaX (0.1)
#define BodyX (0.4)
#define BodyY (0.05)
#define BodyZ (0.15)
#define	DensityOfBody (0.04)

#define NumOfModule (7)		//模块数量
#define DensityOfModule (0.08)
#define NumOfHinge (NumOfModule-1)	//模块间用HingeJoint连接，关节数量

#define ModuleWidth (0.4)//
#define ModuleLength (0.4)
#define ModuleHeight (0.3)//

//“被动轮”相关参数
#define NumOfWheelPair (NumOfModule)	//每个模块1对轮子，轮子数量，转动关节数量
#define	DensityOfWheel (0.001)

#define WheelRadius (0.2)
#define WheelWidth (0.1)

//“管壁”相关参数
#define DensityOfWall (0.98)
#define WallLength (0.1)
#define WallWidth (0.2)
#define WallHeight (0.1)

#define deltaY (0.5)
#define ModuleRadius (0.2)
#define NumOfHeadWheels (1)
#define WheelLength (0.15)

#endif
