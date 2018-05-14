/**************************************/
/*****用于定义程序中使用到的常量值*****/
/**************************************/
#ifndef _VARS2_H_
#define _VARS2_H_

#define LAR (2)
#define PassWheelDensity (0.0001)
#define PassWheelRadius (0.15*LAR)
#define deltaX (0.01*LAR)
#define BodyX (0.4*LAR)
#define BodyY (0.1*LAR)
#define BodyZ (0.15*LAR)
#define	DensityOfBody (0.012)

#define NumOfModule (8)		//模块数量
#define DensityOfModule (0.08)
#define NumOfHinge (NumOfModule-1)	//模块间用HingeJoint连接，关节数量

#define ModuleWidth (0.3*LAR)//
#define ModuleLength (0.4*LAR)
#define ModuleHeight (0.3*LAR)//

//“被动轮”相关参数
#define NumOfWheelPair (NumOfModule)	//每个模块1对轮子，轮子数量，转动关节数量
#define	DensityOfWheel (0.08)

#define WheelRadius (0.2*LAR)
#define WheelWidth (0.1*LAR)

//“管壁”相关参数
#define DensityOfWall (0.98)
#define WallLength (0.1)
#define WallWidth (0.2)
#define WallHeight (0.1)

#define deltaY (0.5*LAR)
#define ModuleRadius (0.2*LAR)
#define NumOfHeadWheels (1*LAR)
#define WheelLength (0.15*LAR)

#define stepsize 0.005

#endif
