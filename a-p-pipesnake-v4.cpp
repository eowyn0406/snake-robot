//----- date: 2018-5-9
//----- version: v4
//----- notes: 修改关节间距使之与实际情况更符合
//----- jointtype of the snake：
//----- head-yaw-body0-pitch-body1-yaw-body2-pitch...-pitch-body7-yaw-tail
//------- 头-偏航-身体0-俯仰-身体1-偏航-身体2-俯仰-...-俯仰-身体7-偏航-尾
//----- add passive wheels
//===========================================================//


#include <ode/ode.h>  //包含动力学仿真的头文件
#include <drawstuff/drawstuff.h>  //包含绘图函数的头文件
#include <math.h>
#include <conio.h>
#include "vars1.h"
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif      //禁止vc++警告

#include<iostream>
#include<fstream>
using namespace std;

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#endif  //如果定义了dDOUBLE型就用双精度绘图函数

//声明一个包含body和geom的类
//typedef struct
class MyObjects
{
	public:
	dBodyID body;
	dGeomID geom;
};

//生成蛇形机器人
MyObjects SnakeBody[NumOfModule];	//生成N个模块
MyObjects SnakeHead;	//生成蛇头模块
MyObjects SnakeTail;	//生成蛇尾模块
MyObjects HeadWheelR[NumOfHeadWheels];	//生成头部主动轮右
MyObjects HeadWheelL[NumOfHeadWheels];	//生成头部主动轮左
MyObjects TailWheelR[NumOfHeadWheels];	//生成尾部主动轮右
MyObjects TailWheelL[NumOfHeadWheels];	//生成尾部主动轮左

//被动轮
MyObjects PassRightWheel[NumOfModule];		//生成N个右边的轮子
MyObjects PassLeftWheel[NumOfModule];		//生成N个左边的轮子

//生成沟渠环境
//MyObjects LeftWall;	//生成两边沟渠
//MyObjects RightWall;	//生成两边沟渠

static dWorldID world;	//定义变量world，类型为dWorldID
static dSpaceID space;	//定义变量space，类型为dSpaceID
static dGeomID  ground;	//定义变量groud，类型为dGeomID
static dJointID BodyJoint[NumOfHinge];	//用来连接各个模块的关节
static dJointID headJoint;	//连接头部和身体的偏航关节
static dJointID tailJoint;	//连接身体和尾部的偏航关节
static dJointID HeadWheelLeftJoint[NumOfHeadWheels];	//连接头部和头部左轮的关节
static dJointID HeadWheelRightJoint[NumOfHeadWheels];	//连接头部和头部右轮的关节
static dJointID TailWheelLeftJoint[NumOfHeadWheels];	//连接尾部和尾部左轮的关节
static dJointID TailWheelRightJoint[NumOfHeadWheels];	//连接尾部和尾部右轮的关节

static dJointID RJ_PassRightWheel[NumOfModule];	//用来连接右边轮子和身体模块的关节
static dJointID RJ_PassLeftWheel[NumOfModule];	//用来连接右边轮子和身体模块的关节

static dJointGroupID contactgroup;	//定义关节组contactgroup，用来存放碰撞检测中需要的特殊关节
static int flag = 0;
dsFunctions fn;			//用于碰撞检测


//子函数声明
static void nearCallback(void *data, dGeomID o1, dGeomID o2);	//碰撞检测
static void start();			//视点设置
static void simLoop(int pause);	//循环
void prepDrawStuff();			//预处理
void jointConnect();			//关节连接
void bodyPositionSet();				//刚体参数设置
void controller(int steps, int ch);
void controller(int steps);
void bodyCreator();	//在world和space中生成各个部分
void headTailCreator();
void environmentSet();


void headTailCreator()
{
	SnakeHead.body = dBodyCreate(world);
	SnakeTail.body = dBodyCreate(world);
	SnakeHead.geom = dCreateBox(space,ModuleWidth/2,ModuleWidth,ModuleHeight);
	SnakeTail.geom = dCreateBox(space,ModuleWidth/2,ModuleWidth,ModuleHeight);
	dGeomSetBody(SnakeHead.geom,SnakeHead.body);
	dGeomSetBody(SnakeTail.geom,SnakeTail.body);
			

	dMass m;
	dMassSetZero (&m);			//设置质量为零
	dMatrix3 Sp;

	dRFrom2Axes(Sp,1,0,0,0,1,0);	//分别以世界坐标系的X,Y轴作为模块本体的X,Y轴
	dMassSetBox(&m,DensityOfModule,ModuleWidth,ModuleWidth,ModuleHeight);

	dBodySetMass (SnakeHead.body,&m);
	dReal CenHead[] = {BodyX/2+ModuleLength/2+deltaX/2,deltaY,WheelRadius};
	dBodySetPosition(SnakeHead.body,CenHead[0],CenHead[1],CenHead[2]);
	dBodySetRotation(SnakeHead.body,Sp);

	dBodySetMass (SnakeTail.body,&m);
	dReal CenTail[] = {-(BodyX+deltaX)*NumOfModule+(BodyX+deltaX)/2-ModuleLength/2,deltaY,WheelRadius};
	dBodySetPosition(SnakeTail.body,CenTail[0],CenTail[1],CenTail[2]);
	dBodySetRotation(SnakeTail.body,Sp);

	//生成侧边排轮
	for(int i=0; i<NumOfHeadWheels; i++)
	{
		HeadWheelR[i].body = dBodyCreate(world);
		HeadWheelL[i].body = dBodyCreate(world);
		HeadWheelR[i].geom = dCreateCylinder(space,WheelRadius,WheelLength);
		HeadWheelL[i].geom = dCreateCylinder(space,WheelRadius,WheelLength);

		TailWheelR[i].body = dBodyCreate(world);
		TailWheelL[i].body = dBodyCreate(world);
		TailWheelR[i].geom = dCreateCylinder(space,WheelRadius,WheelLength);
		TailWheelL[i].geom = dCreateCylinder(space,WheelRadius,WheelLength);

		dGeomSetBody(HeadWheelR[i].geom,HeadWheelR[i].body);
		dGeomSetBody(HeadWheelL[i].geom,HeadWheelL[i].body);
		dGeomSetBody(TailWheelR[i].geom,TailWheelR[i].body);
		dGeomSetBody(TailWheelL[i].geom,TailWheelL[i].body);

		dMassSetCylinder(&m,DensityOfWheel,3,WheelRadius,WheelLength);

		dBodySetMass (HeadWheelL[i].body,&m);
		dBodySetPosition(HeadWheelL[i].body,CenHead[0],CenHead[1]+ModuleWidth/2+WheelLength/2+0.01, WheelRadius);

		//dBodySetPosition(HeadWheelL[i].body,sqrt(2)*ModuleWidth/2+7*ModuleLength/4-0.25*i*ModuleLength,deltaY+ModuleWidth/2+WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dRFrom2Axes(Sp,0,0,1,1,0,0);	//分别以世界坐标系的Z,X轴作为轮子本体的X,Y轴
		dBodySetRotation(HeadWheelL[i].body,Sp);

		dBodySetMass (HeadWheelR[i].body,&m);
		dBodySetPosition(HeadWheelR[i].body,CenHead[0],CenHead[1]-ModuleWidth/2-WheelLength/2-0.01, WheelRadius);
		
		//dBodySetPosition(HeadWheelR[i].body,sqrt(2)*ModuleWidth/2+7*ModuleLength/4-0.25*i*ModuleLength,deltaY-ModuleWidth/2-WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dRFrom2Axes(Sp,0,0,1,1,0,0);	//分别以世界坐标系的Z,X轴作为轮子本体的X,Y轴
		dBodySetRotation(HeadWheelR[i].body,Sp);

		dBodySetMass (TailWheelL[i].body,&m);
		dBodySetPosition(TailWheelL[i].body,CenTail[0],CenTail[1]+ModuleWidth/2+WheelLength/2+0.01, WheelRadius);

		//dBodySetPosition(TailWheelL[i].body,-2*NumOfModule*BodyLength-sqrt(2)*ModuleWidth/2+3*ModuleLength/4-0.25*i*ModuleLength,deltaY+ModuleWidth/2+WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dRFrom2Axes(Sp,0,0,1,1,0,0);	//分别以世界坐标系的Z,X轴作为轮子本体的X,Y轴
		dBodySetRotation(TailWheelL[i].body,Sp);

		dBodySetMass (TailWheelR[i].body,&m);
		dBodySetPosition(TailWheelR[i].body,CenTail[0],CenTail[1]-ModuleWidth/2-WheelLength/2-0.01, WheelRadius);
		
		//dBodySetPosition(TailWheelR[i].body,-2*NumOfModule*BodyLength-sqrt(2)*ModuleWidth/2+3*ModuleLength/4-0.25*i*ModuleLength,deltaY-ModuleWidth/2-WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dRFrom2Axes(Sp,0,0,1,1,0,0);	//分别以世界坐标系的Z,X轴作为轮子本体的X,Y轴
		dBodySetRotation(TailWheelR[i].body,Sp);

	}

	//添加头尾主动轮关节
	dReal deltay = WheelLength/2+0.001;
	for(int i=0; i<NumOfHeadWheels; i++)
	{
		HeadWheelLeftJoint[i] = dJointCreateHinge(world,0);
		HeadWheelRightJoint[i] = dJointCreateHinge(world,0);
		TailWheelLeftJoint[i] = dJointCreateHinge(world,0);
		TailWheelRightJoint[i] = dJointCreateHinge(world,0);
		dJointAttach(HeadWheelLeftJoint[i],SnakeHead.body,HeadWheelL[i].body);
		dJointAttach(HeadWheelRightJoint[i],SnakeHead.body,HeadWheelR[i].body);
		dJointAttach(TailWheelLeftJoint[i],SnakeTail.body,TailWheelL[i].body);
		dJointAttach(TailWheelRightJoint[i],SnakeTail.body,TailWheelR[i].body);
		
		dJointSetHingeAnchor(HeadWheelLeftJoint[i],CenHead[0],deltaY+ModuleWidth/2+deltay, WheelRadius);
		dJointSetHingeAxis(HeadWheelLeftJoint[i],0,1,0);
		dJointSetHingeAnchor(HeadWheelRightJoint[i],CenHead[0],deltaY-ModuleWidth/2-deltay, WheelRadius);
		dJointSetHingeAxis(HeadWheelRightJoint[i],0,1,0);

		dJointSetHingeAnchor(TailWheelLeftJoint[i],CenTail[0],deltaY+ModuleWidth/2+deltay, WheelRadius);
		dJointSetHingeAxis(TailWheelLeftJoint[i],0,1,0);
		dJointSetHingeAnchor(TailWheelRightJoint[i],CenTail[0],deltaY-ModuleWidth/2-deltay, WheelRadius);
		dJointSetHingeAxis(TailWheelRightJoint[i],0,1,0);
	}
	
}

void bodyCreator()
{
	for(int i=0;i<NumOfModule;i++)
	{
		SnakeBody[i].body = dBodyCreate (world);		//生成各个模块
		PassRightWheel[i].body = dBodyCreate (world);
		PassRightWheel[i].geom = dCreateCylinder(space,PassWheelRadius,WheelLength);
		PassLeftWheel[i].body = dBodyCreate (world);
		PassLeftWheel[i].geom = dCreateCylinder(space,PassWheelRadius,WheelLength);
		//if(i%2==0)
			SnakeBody[i].geom = dCreateBox(space,BodyX/2,BodyY,BodyZ);//生成几何形状
		//else
			//SnakeBody[i].geom = dCreateBox(space,BodyY,BodyX,BodyZ);//生成几何形状
		dGeomSetBody(SnakeBody[i].geom,SnakeBody[i].body);//添加关联geom和body
		dGeomSetBody(PassRightWheel[i].geom,SnakeBody[i].body);//添加关联geom和body
		dGeomSetBody(PassLeftWheel[i].geom,SnakeBody[i].body);//添加关联geom和body
	}

	bodyPositionSet();
}

//位姿设置函数,包括质量，几何形状等的设置

void bodyPositionSet()
{
	dMatrix3 Sp_SnakeBody[NumOfModule];	//每个模块的位姿矩阵
	dMass m;
	dMassSetZero (&m);			//设置质量为零
	dMatrix3 passwheel;
	dRFrom2Axes(passwheel,0,0,1,1,0,0);
	for(int i=0;i<NumOfModule;i++)
	{
		dMassSetCylinder (&m,PassWheelDensity,3,PassWheelRadius,WheelLength);
		dBodySetMass(PassRightWheel[i].body,&m);
		dBodySetPosition(PassRightWheel[i].body,-BodyX*i-i*deltaX,-BodyY/2-WheelLength/2-0.01+deltaY,PassWheelRadius);
		dBodySetRotation(PassRightWheel[i].body,passwheel);
		dBodySetMass(PassLeftWheel[i].body,&m);
		dBodySetPosition(PassLeftWheel[i].body,-BodyX*i-i*deltaX,BodyY/2+WheelLength/2+0.01+deltaY,PassWheelRadius);
		dBodySetRotation(PassLeftWheel[i].body,passwheel);
	}
	for(int i=0;i<NumOfModule;i++)
	{
		dRFrom2Axes(Sp_SnakeBody[i],1,0,0,0,1,0);	//分别以世界坐标系的X,Y轴作为模块本体的X,Y轴
		//if(i%2==0)
			dMassSetBox(&m,DensityOfBody,BodyX,BodyY,BodyZ);
		//else
			//dMassSetBox(&m,DensityOfBody,BodyX,BodyZ,BodyY);
		dBodySetMass (SnakeBody[i].body,&m);
		dBodySetPosition(SnakeBody[i].body,-BodyX*i-i*deltaX,0+deltaY,PassWheelRadius);
		dBodySetRotation(SnakeBody[i].body,Sp_SnakeBody[i]);
	}
}

void environmentSet()
{
	//生成两面墙体
	/*LeftWall.body = dBodyCreate(world);
	LeftWall.geom = dCreateBox(space, WallLength, WallWidth, WallHeight);
	dGeomSetBody(LeftWall.geom, LeftWall.body);
	RightWall.body = dBodyCreate(world);
	RightWall.geom = dCreateBox(space, WallLength, WallWidth, WallHeight);
	dGeomSetBody(RightWall.geom, RightWall.body);*/

	/*dRFrom2Axes(Wall, 1, 0, 0, 0, 1, 0);  //分别以世界坐标系的X,Y轴作为墙体的X,Y轴
	dMassSetBox(&m, DensityOfWall, WallLength, WallWidth, WallHeight);
	dBodySetMass(LeftWall.body, &m);
	dBodySetPosition(LeftWall.body, WallLength/2, -(WallWidth/2 + ModuleLength) + deltaY, WallHeight / 2);
	dBodySetRotation(LeftWall.body, Wall);
	dBodySetMass(RightWall.body, &m);
	dBodySetPosition(RightWall.body, WallLength/2, WallWidth / 2 + ModuleLength + deltaY, WallHeight / 2);
	dBodySetRotation(RightWall.body, Wall);*/
}


//关节连接函数
void jointConnect()
{
	//连接被动模块的关节：8个模块、7个关节。
	//头-偏航-身体0-俯仰-身体1-偏航-身体2-俯仰-...-俯仰-身体7-偏航-尾
	for(int i=0;i<NumOfHinge;i++)
	{
		BodyJoint[i] = dJointCreateHinge(world,0);
		dJointAttach(BodyJoint[i],SnakeBody[i].body,SnakeBody[i+1].body);	//	连接第i个模块和第i+1个模块
		dJointSetHingeAnchor(BodyJoint[i],-BodyX/2-deltaX/2-BodyX*i-i*deltaX,0+deltaY,PassWheelRadius);
		if(i%2 == 0)
			dJointSetHingeAxis(BodyJoint[i],0,1,0);	//以Y轴作为偶数驱动关节的旋转轴：俯仰自由度
		else
			dJointSetHingeAxis(BodyJoint[i],0,0,1);	//以Z轴作为奇数驱动关节的旋转轴：偏航自由度
		dJointSetHingeParam(BodyJoint[i], dParamLoStop, -M_PI/2);
		dJointSetHingeParam(BodyJoint[i], dParamHiStop, M_PI/2);
		dJointSetHingeParam(BodyJoint[i],dParamBounce,0.01);	//保证关节为刚性关节
	}
	
	for(int i=0;i<NumOfModule;i++)
	{
		RJ_PassRightWheel[i] = dJointCreateHinge(world,0);
		RJ_PassLeftWheel[i] = dJointCreateHinge(world,0);

		dJointAttach(RJ_PassRightWheel[i],SnakeBody[i].body,PassRightWheel[i].body);
		dJointSetHingeAnchor(RJ_PassRightWheel[i],-BodyX*i-i*deltaX,-BodyY/2+deltaY,PassWheelRadius);
		dJointSetHingeAxis(RJ_PassRightWheel[i],0,1,0);
		
		dJointAttach(RJ_PassLeftWheel[i],SnakeBody[i].body,PassLeftWheel[i].body);
		dJointSetHingeAnchor(RJ_PassLeftWheel[i],-BodyX*i-i*deltaX,BodyY/2+deltaY,PassWheelRadius);
		dJointSetHingeAxis(RJ_PassLeftWheel[i],0,1,0);

	}
	 

	//添加头尾模块偏航关节
	headJoint = dJointCreateHinge(world,0);
	tailJoint = dJointCreateHinge(world,0);
	dJointAttach(headJoint,SnakeHead.body,SnakeBody[0].body);	//	连接第1个模块和头模块
	dJointAttach(tailJoint,SnakeBody[NumOfModule-1].body,SnakeTail.body);	//	连接第8个模块和尾模块
	dJointSetHingeAnchor(headJoint,0,0+deltaY,ModuleHeight/2);
	dJointSetHingeAnchor(tailJoint,-ModuleLength/2-(BodyX+deltaX)*NumOfHinge+(BodyX+deltaX)/2,0+deltaY,ModuleHeight/2);
	dJointSetHingeAxis(headJoint,0,0,1);	//以Z轴作为驱动关节的旋转轴
	dJointSetHingeAxis(tailJoint,0,0,1);	//以Z轴作为驱动关节的旋转轴
	dJointSetHingeParam(headJoint, dParamLoStop, -M_PI/2);
	dJointSetHingeParam(headJoint, dParamHiStop, M_PI/2);
	dJointSetHingeParam(tailJoint, dParamLoStop, -M_PI/2);
	dJointSetHingeParam(tailJoint, dParamHiStop, M_PI/2);
	dJointSetHingeParam(headJoint,dParamBounce,0.01);//保证关节为刚性关节
	dJointSetHingeParam(tailJoint,dParamBounce,0.01);//保证关节为刚性关节


	
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 10;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

  if (isGround)  {
		if (n >= 1) flag = 1;
    else        flag = 0;
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode = dContactBounce|dContactSoftERP|dContactSoftCFM;//
      contact[i].surface.mu   = 0.5;
      contact[i].surface.bounce     = 0; // (0.0~1.0) restitution parameter
	  contact[i].surface.soft_erp = 0.8; //
	  contact[i].surface.soft_cfm = 0.5; //
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
    }
  }
}

static void start()
{
  static float xyz[3] = {0,10,10};
  static float hpr[3] = {-90, -30, 0};
  dsSetViewpoint (xyz,hpr);				// 设定视点和视线
}

//循环程序，每一步都执行一次

static void simLoop(int pause)
{
	static int steps = 0;
	int ch = 0;
	//ofstream out("data.txt", ios::app);
	//ofstream out2("data1.txt", ios::app);
	//const dReal *pSnake;
	//dReal ya, yb;
	/*for (int i = 0; i < NumOfModule; i++)
	{	
		pSnake = dBodyGetPosition(SnakeBody[i].body);
		out << "module " << i << ", z: " << pSnake[2] << endl;
		if (i == 0) ya = pSnake[0];
		if (i == NumOfModule-1) yb = pSnake[0];
	}
	out2 << yb - ya<<endl;*/
	

	const dReal *spos[NumOfModule],*sR[NumOfModule];
	const dReal *rwpos[NumOfWheelPair],*rwR[NumOfWheelPair];
	const dReal *lwpos[NumOfWheelPair],*lwR[NumOfWheelPair];
	const dReal *ap, *ar;
	const dReal *htp, *htr;

	flag = 0;
	dSpaceCollide(space,0,&nearCallback);
	dWorldStepFast1(world,0.005,20);
	dJointGroupEmpty(contactgroup);



	int i;
	//dReal sides[3] = {BodyLength,ModuleWidth,ModuleHeight};
	//dReal WallSize[3] = {WallLength,WallWidth,WallHeight};
	for(i=0;i<NumOfModule;i++)
	{
		dsSetColorAlpha(1,1,1,1);	//设置模块的颜色
		spos[i] = dBodyGetPosition(SnakeBody[i].body);		//获得模块当前的位置
		sR[i]   = dBodyGetRotation(SnakeBody[i].body);		//获得模块当前的姿态
		dReal SidesEven[3] = {BodyX,BodyY,BodyZ};
		//dReal SidesOdd[3] = {BodyX,BodyZ,BodyY};
		//if(i%2==0)
			dsDrawBox(spos[i],sR[i],SidesEven);	//用获得的位姿和长度、半径等绘制当前的模块
		//else
			//dsDrawBox(spos[i],sR[i],SidesOdd);	//用获得的位姿和长度、半径等绘制当前的模块
	}

	for(i=0;i<NumOfHeadWheels;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//设置模块的颜色
		spos[i] = dBodyGetPosition(HeadWheelL[i].body);		//获得模块当前的位置
		sR[i]   = dBodyGetRotation(HeadWheelL[i].body);		//获得模块当前的姿态
		dsDrawCylinder(spos[i],sR[i],WheelLength,WheelRadius);	//用获得的位姿和长度、半径等绘制当前的模块
	}

	for(i=0;i<NumOfHeadWheels;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//设置模块的颜色
		spos[i] = dBodyGetPosition(HeadWheelR[i].body);		//获得模块当前的位置
		sR[i]   = dBodyGetRotation(HeadWheelR[i].body);		//获得模块当前的姿态
		dsDrawCylinder(spos[i],sR[i],WheelLength,WheelRadius);	//用获得的位姿和长度、半径等绘制当前的模块
	}

	for(i=0;i<NumOfHeadWheels;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//设置模块的颜色
		spos[i] = dBodyGetPosition(TailWheelL[i].body);		//获得模块当前的位置
		sR[i]   = dBodyGetRotation(TailWheelL[i].body);		//获得模块当前的姿态
		dsDrawCylinder(spos[i],sR[i],WheelLength,WheelRadius);	//用获得的位姿和长度、半径等绘制当前的模块
	}

	for(i=0;i<NumOfHeadWheels;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//设置模块的颜色
		spos[i] = dBodyGetPosition(TailWheelR[i].body);		//获得模块当前的位置
		sR[i]   = dBodyGetRotation(TailWheelR[i].body);		//获得模块当前的姿态
		dsDrawCylinder(spos[i],sR[i],WheelLength,WheelRadius);	//用获得的位姿和长度、半径等绘制当前的模块
	}

	dReal sidess[3] = {ModuleWidth,ModuleWidth,ModuleHeight};
	dsSetColorAlpha(1,1,1,0.8);	//设置头尾模块的颜色
	ap = dBodyGetPosition(SnakeHead.body);		//获得模块当前的位置
	ar = dBodyGetRotation(SnakeHead.body);		//获得模块当前的姿态
	dsDrawBox(ap,ar,sidess);	//用获得的位姿和长度、半径等绘制当前的模块
	ap = dBodyGetPosition(SnakeTail.body);		//获得模块当前的位置
	ar = dBodyGetRotation(SnakeTail.body);		//获得模块当前的姿态
	dsDrawBox(ap,ar,sidess);	//用获得的位姿和长度、半径等绘制当前的模块
	
	for(int i=0;i<NumOfModule;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//设置模块的颜色
		spos[i] = dBodyGetPosition(PassLeftWheel[i].body);		//获得模块当前的位置
		sR[i]   = dBodyGetRotation(PassLeftWheel[i].body);		//获得模块当前的姿态
		dsDrawCylinder(spos[i],sR[i],WheelLength,PassWheelRadius);	//用获得的位姿和长度、半径等绘制当前的模块
		spos[i] = dBodyGetPosition(PassRightWheel[i].body);		//获得模块当前的位置
		sR[i]   = dBodyGetRotation(PassRightWheel[i].body);		//获得模块当前的姿态
		dsDrawCylinder(spos[i],sR[i],WheelLength,PassWheelRadius);	//用获得的位姿和长度、半径等绘制当前的模块
	}

	//controller(steps++);
	//const dReal* Head;
	//Head = dBodyGetPosition(SnakeHead.body);
	//ofstream out("snakehead.txt",ios::app);
	//out<<Head[0]<<" "<<Head[1]<<" "<<Head[2]<<endl;
	
	printf("%d\n",steps);
	if(steps>2)
		while(1)
		{
			ch = getch();
			if(ch == 119 || ch == 115 || ch == 97 || ch == 100)
				break;
		}// w前进，s后退，a左转，d右转
	controller(steps++,ch);

}


void  prepDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.stop    = NULL;
  fn.path_to_textures = "../../drawstuff/textures";
}



int main (int argc, char **argv)
{
	prepDrawStuff();
	

	world = dWorldCreate();//生成动力学计算用的world
	space = dHashSpaceCreate(0);//生成碰撞检测用的space
	contactgroup = dJointGroupCreate(0);//生成碰撞检测用的特殊关节
	dWorldSetGravity(world,0,0,-9.81);//设为地球重力加速度
	ground = dCreatePlane(space,0,0,1,0);//在space中创建了z=0平面，用于碰撞检测

	dWorldSetERP(world,0.2);//设置ERP参数
	dWorldSetCFM(world,0.5);//设置CFM参数
	
	headTailCreator();
	bodyCreator();
	jointConnect();

	dWorldSetContactSurfaceLayer(world,0.001);

	ofstream out("snakeposition.txt",ios::app);
	const dReal* sp;
	sp = dBodyGetPosition(SnakeBody[0].body);
	out<<"position: "<<sp[0]<<" "<<sp[1]<<" "<<sp[2]<<endl;
	sp = dBodyGetRotation(SnakeBody[0].body);
	out<<"rotation: "<<sp[0]<<" "<<sp[1]<<" "<<sp[2]<<endl;
	out<<"rotation: "<<sp[3]<<" "<<sp[4]<<" "<<sp[5]<<endl;
	out<<"rotation: "<<sp[6]<<" "<<sp[7]<<" "<<sp[8]<<endl;
	out<<"rotation: "<<sp[9]<<" "<<sp[10]<<" "<<sp[11]<<endl;

	dsSimulationLoop (argc,argv,960,480,&fn);
	dWorldDestroy (world);
	dCloseODE();

	
	//out.close();
	return 0;

}

void controller(int steps, int ch)
{
	float desire_angle[NumOfHinge+2] = {};	//目标角度{BodyJoint[0],...,BodyJoint[NumOfHinge-1],headJoint,tailJoint}
	float diff_angle[NumOfHinge+2] = {};		//当前角度-目标角度
	float maxtorque = 1000;

	if(steps>100 && steps<4000)
	{
		//跟随蛇形曲线轨迹
		for(int i=0; i<NumOfHinge; i++)
		{
			//if(i%2==0)
			//	desire_angle[i] = M_PI/1000;
			//else
				desire_angle[i] = 0;
		}
		desire_angle[NumOfHinge] = 0;//头部偏航角度 M_PI/18000*steps;
		desire_angle[NumOfHinge+1] = 0;//尾部偏航角度
		
		int HLvel = 0;
		int HRvel = 0;
		int TLvel = 0;
		int TRvel = 0;
		
		switch (ch)
		{//119w,115s,97a,100d
		case 119:
			HLvel = -20;
			HRvel = -20;
			TLvel = 0;
			TRvel = 0;
			break;
		case 115:
			HLvel = 0;
			HRvel = 0;
			TLvel = 20;
			TRvel = 20;
			break;
		case 97:
			HLvel = -10;
			HRvel = -20;
			TLvel = -10;
			TRvel = -10;
			break;
		case 100:
			HLvel = -20;
			HRvel = -10;
			TLvel = -10;
			TRvel = -10;
			break;
		}
			

		for(int i=0; i<NumOfHeadWheels; i++)
		{
			dJointSetHingeParam(HeadWheelLeftJoint[i],dParamVel,HLvel);
			dJointSetHingeParam(HeadWheelLeftJoint[i],dParamFMax,maxtorque);
			dJointSetHingeParam(HeadWheelRightJoint[i],dParamVel,HRvel);
			dJointSetHingeParam(HeadWheelRightJoint[i],dParamFMax,maxtorque);
			dJointSetHingeParam(TailWheelLeftJoint[i],dParamVel,TLvel);
			dJointSetHingeParam(TailWheelLeftJoint[i],dParamFMax,maxtorque);
			dJointSetHingeParam(TailWheelRightJoint[i],dParamVel,TRvel);
			dJointSetHingeParam(TailWheelRightJoint[i],dParamFMax,maxtorque);
		}	
	}
	else
	{
		for(int i=0; i<NumOfHinge+2; i++)
		{
			desire_angle[i] = 0;
		}
		for(int i=0; i<NumOfHeadWheels; i++)
		{
			dJointSetHingeParam(HeadWheelLeftJoint[i],dParamVel,0);
			dJointSetHingeParam(HeadWheelLeftJoint[i],dParamFMax,maxtorque);
			dJointSetHingeParam(HeadWheelRightJoint[i],dParamVel,0);
			dJointSetHingeParam(HeadWheelRightJoint[i],dParamFMax,maxtorque);
			dJointSetHingeParam(TailWheelLeftJoint[i],dParamVel,0);
			dJointSetHingeParam(TailWheelLeftJoint[i],dParamFMax,maxtorque);
			dJointSetHingeParam(TailWheelRightJoint[i],dParamVel,0);
			dJointSetHingeParam(TailWheelRightJoint[i],dParamFMax,maxtorque);
		}	
	}

	for(int i=0; i<NumOfHinge+2; i++)
	{
		diff_angle[i] = desire_angle[i] - dJointGetHingeAngle(BodyJoint[i]);
		dJointSetHingeParam(BodyJoint[i],dParamVel,10*diff_angle[i]);
		dJointSetHingeParam(BodyJoint[i],dParamFMax,maxtorque);
	}
	diff_angle[NumOfHinge] = desire_angle[NumOfHinge] - dJointGetHingeAngle(headJoint);
	dJointSetHingeParam(headJoint,dParamVel,10*diff_angle[NumOfHinge]);
	dJointSetHingeParam(headJoint,dParamFMax,maxtorque);
	diff_angle[NumOfHinge+1] = desire_angle[NumOfHinge+1] - dJointGetHingeAngle(tailJoint);
	dJointSetHingeParam(tailJoint,dParamVel,10*diff_angle[NumOfHinge+1]);
	dJointSetHingeParam(tailJoint,dParamFMax,maxtorque);

}
