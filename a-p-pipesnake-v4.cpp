//----- date: 2018-5-9
//----- version: v4
//----- notes: �޸Ĺؽڼ��ʹ֮��ʵ�����������
//----- jointtype of the snake��
//----- head-yaw-body0-pitch-body1-yaw-body2-pitch...-pitch-body7-yaw-tail
//------- ͷ-ƫ��-����0-����-����1-ƫ��-����2-����-...-����-����7-ƫ��-β
//===========================================================//


#include <ode/ode.h>  //��������ѧ�����ͷ�ļ�
#include <drawstuff/drawstuff.h>  //������ͼ������ͷ�ļ�
#include <math.h>
#include <conio.h>
#include "vars1.h"
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif      //��ֹvc++����

#include<iostream>
#include<fstream>
using namespace std;

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#endif  //���������dDOUBLE�;���˫���Ȼ�ͼ����

//����һ������body��geom����
//typedef struct
class MyObjects
{
	public:
	dBodyID body;
	dGeomID geom;
};

//�������λ�����
MyObjects SnakeBody[NumOfModule];	//����N��ģ��
MyObjects SnakeHead;	//������ͷģ��
MyObjects SnakeTail;	//������βģ��
MyObjects HeadWheelR[NumOfHeadWheels];	//����ͷ����������
MyObjects HeadWheelL[NumOfHeadWheels];	//����ͷ����������
MyObjects TailWheelR[NumOfHeadWheels];	//����β����������
MyObjects TailWheelL[NumOfHeadWheels];	//����β����������

//MyObjects RightWheel[NumOfWheelPair];		//����N���ұߵ�����
//MyObjects LeftWheel[NumOfWheelPair];		//����N����ߵ�����

//���ɹ�������
//MyObjects LeftWall;	//�������߹���
//MyObjects RightWall;	//�������߹���

static dWorldID world;	//�������world������ΪdWorldID
static dSpaceID space;	//�������space������ΪdSpaceID
static dGeomID  ground;	//�������groud������ΪdGeomID
static dJointID BodyJoint[NumOfHinge];	//�������Ӹ���ģ��Ĺؽ�
static dJointID headJoint;	//����ͷ���������ƫ���ؽ�
static dJointID tailJoint;	//���������β����ƫ���ؽ�
static dJointID HeadWheelLeftJoint[NumOfHeadWheels];
static dJointID HeadWheelRightJoint[NumOfHeadWheels];
static dJointID TailWheelLeftJoint[NumOfHeadWheels];
static dJointID TailWheelRightJoint[NumOfHeadWheels];

//static dJointID RotateJoint_RightWheel[NumOfWheelPair];	//���������ұ����Ӻ�ģ��Ĺؽ�
//static dJointID RotateJoint_LeftWheel[NumOfWheelPair];	//���������ұ����Ӻ�ģ��Ĺؽ�

static dJointGroupID contactgroup;	//����ؽ���contactgroup�����������ײ�������Ҫ������ؽ�
static int flag = 0;
dsFunctions fn;			//������ײ���


//�Ӻ�������
static void nearCallback(void *data, dGeomID o1, dGeomID o2);	//��ײ���
static void start();			//�ӵ�����
static void simLoop(int pause);	//ѭ��
void prepDrawStuff();			//Ԥ����
void jointConnect();			//�ؽ�����
void bodyPositionSet();				//�����������
void controller(int steps, int ch);
void controller(int steps);
void bodyCreator();	//��world��space�����ɸ�������
void headTailCreator();
void environmentSet();


void headTailCreator()
{
	SnakeHead.body = dBodyCreate(world);
	SnakeTail.body = dBodyCreate(world);
	SnakeHead.geom = dCreateBox(space,ModuleWidth*2,ModuleWidth,ModuleHeight);
	SnakeTail.geom = dCreateBox(space,ModuleWidth*2,ModuleWidth,ModuleHeight);
	dGeomSetBody(SnakeHead.geom,SnakeHead.body);
	dGeomSetBody(SnakeTail.geom,SnakeTail.body);
			
	dMass m;
	dMassSetZero (&m);			//��������Ϊ��
	dMatrix3 Sp;

	dRFrom2Axes(Sp,1,0,0,0,1,0);	//�ֱ�����������ϵ��X,Y����Ϊģ�鱾���X,Y��
	dMassSetBox(&m,DensityOfModule,ModuleWidth,ModuleWidth,ModuleHeight);

	dBodySetMass (SnakeHead.body,&m);
	dReal CenHead[] = {ModuleWidth+BodyLength/2+ModuleLength/2,deltaY,ModuleHeightCen};
	dBodySetPosition(SnakeHead.body,CenHead[0],CenHead[1],CenHead[2]);
	dBodySetRotation(SnakeHead.body,Sp);

	dBodySetMass (SnakeTail.body,&m);
	dReal CenTail[] = {-BodyLength*2*NumOfModule-ModuleWidth+BodyLength/2,deltaY,ModuleHeightCen};
	dBodySetPosition(SnakeTail.body,CenTail[0],CenTail[1],CenTail[2]);
	dBodySetRotation(SnakeTail.body,Sp);

	//���ɲ������
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
		dBodySetPosition(HeadWheelL[i].body,CenHead[0],deltaY+ModuleWidth/2+WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);

		//dBodySetPosition(HeadWheelL[i].body,sqrt(2)*ModuleWidth/2+7*ModuleLength/4-0.25*i*ModuleLength,deltaY+ModuleWidth/2+WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dRFrom2Axes(Sp,0,0,1,1,0,0);	//�ֱ�����������ϵ��Z,X����Ϊ���ӱ����X,Y��
		dBodySetRotation(HeadWheelL[i].body,Sp);

		dBodySetMass (HeadWheelR[i].body,&m);
		dBodySetPosition(HeadWheelR[i].body,CenHead[0],deltaY-ModuleWidth/2-WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		
		//dBodySetPosition(HeadWheelR[i].body,sqrt(2)*ModuleWidth/2+7*ModuleLength/4-0.25*i*ModuleLength,deltaY-ModuleWidth/2-WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dRFrom2Axes(Sp,0,0,1,1,0,0);	//�ֱ�����������ϵ��Z,X����Ϊ���ӱ����X,Y��
		dBodySetRotation(HeadWheelR[i].body,Sp);

		dBodySetMass (TailWheelL[i].body,&m);
		dBodySetPosition(TailWheelL[i].body,CenTail[0],deltaY+ModuleWidth/2+WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);

		//dBodySetPosition(TailWheelL[i].body,-2*NumOfModule*BodyLength-sqrt(2)*ModuleWidth/2+3*ModuleLength/4-0.25*i*ModuleLength,deltaY+ModuleWidth/2+WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dRFrom2Axes(Sp,0,0,1,1,0,0);	//�ֱ�����������ϵ��Z,X����Ϊ���ӱ����X,Y��
		dBodySetRotation(TailWheelL[i].body,Sp);

		dBodySetMass (TailWheelR[i].body,&m);
		dBodySetPosition(TailWheelR[i].body,CenTail[0],deltaY-ModuleWidth/2-WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		
		//dBodySetPosition(TailWheelR[i].body,-2*NumOfModule*BodyLength-sqrt(2)*ModuleWidth/2+3*ModuleLength/4-0.25*i*ModuleLength,deltaY-ModuleWidth/2-WheelLength/2, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dRFrom2Axes(Sp,0,0,1,1,0,0);	//�ֱ�����������ϵ��Z,X����Ϊ���ӱ����X,Y��
		dBodySetRotation(TailWheelR[i].body,Sp);

	}

	//���ͷβ�����ֹؽ�
	dReal deltay = 0.001;
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
		
		dJointSetHingeAnchor(HeadWheelLeftJoint[i],CenHead[0],deltaY+ModuleWidth/2+deltay, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dJointSetHingeAxis(HeadWheelLeftJoint[i],0,1,0);
		dJointSetHingeAnchor(HeadWheelRightJoint[i],CenHead[0],deltaY-ModuleWidth/2-deltay, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dJointSetHingeAxis(HeadWheelRightJoint[i],0,1,0);

		dJointSetHingeAnchor(TailWheelLeftJoint[i],CenTail[0],deltaY+ModuleWidth/2+deltay, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dJointSetHingeAxis(TailWheelLeftJoint[i],0,1,0);
		dJointSetHingeAnchor(TailWheelRightJoint[i],CenTail[0],deltaY-ModuleWidth/2-deltay, ModuleHeightCen-ModuleHeight/2+WheelRadius*7/8);
		dJointSetHingeAxis(TailWheelRightJoint[i],0,1,0);
	}
	
}

void bodyCreator()
{
	for(int i=0;i<NumOfModule;i++)
	{
		SnakeBody[i].body = dBodyCreate (world);		//���ɸ���ģ��
		if(i%2==0)
			SnakeBody[i].geom = dCreateBox(space,BodyX,BodyY,BodyZ);//���ɼ�����״
		else
			SnakeBody[i].geom = dCreateBox(space,BodyY,BodyX,BodyZ);//���ɼ�����״
		dGeomSetBody(SnakeBody[i].geom,SnakeBody[i].body);//��ӹ���geom��body
	}

	bodyPositionSet();
}

//λ�����ú���,����������������״�ȵ�����

void bodyPositionSet()
{
	dMatrix3 Sp_SnakeBody[NumOfModule];	//ÿ��ģ���λ�˾���
	dMass m;
	dMassSetZero (&m);			//��������Ϊ��
	for(int i=0;i<NumOfModule;i++)
	{
		dRFrom2Axes(Sp_SnakeBody[i],1,0,0,0,1,0);	//�ֱ�����������ϵ��X,Y����Ϊģ�鱾���X,Y��
		if(i%2==0)
			dMassSetBox(&m,DensityOfBody,BodyX,BodyY,BodyZ);
		else
			dMassSetBox(&m,DensityOfBody,BodyY,BodyX,BodyZ);
		dBodySetMass (SnakeBody[i].body,&m);
		dBodySetPosition(SnakeBody[i].body,-BodyLength*2*i,0+deltaY,ModuleHeightCen);
		dBodySetRotation(SnakeBody[i].body,Sp_SnakeBody[i]);
	}
}

void environmentSet()
{
		//��������ǽ��
	/*LeftWall.body = dBodyCreate(world);
	LeftWall.geom = dCreateBox(space, WallLength, WallWidth, WallHeight);
	dGeomSetBody(LeftWall.geom, LeftWall.body);
	RightWall.body = dBodyCreate(world);
	RightWall.geom = dCreateBox(space, WallLength, WallWidth, WallHeight);
	dGeomSetBody(RightWall.geom, RightWall.body);*/

	/*dRFrom2Axes(Wall, 1, 0, 0, 0, 1, 0);  //�ֱ�����������ϵ��X,Y����Ϊǽ���X,Y��
	dMassSetBox(&m, DensityOfWall, WallLength, WallWidth, WallHeight);
	dBodySetMass(LeftWall.body, &m);
	dBodySetPosition(LeftWall.body, WallLength/2, -(WallWidth/2 + ModuleLength) + deltaY, WallHeight / 2);
	dBodySetRotation(LeftWall.body, Wall);
	dBodySetMass(RightWall.body, &m);
	dBodySetPosition(RightWall.body, WallLength/2, WallWidth / 2 + ModuleLength + deltaY, WallHeight / 2);
	dBodySetRotation(RightWall.body, Wall);*/
}


//�ؽ����Ӻ���
void jointConnect()
{
	//���ӱ���ģ��Ĺؽڣ�8��ģ�顢7���ؽڡ�
	//ͷ-ƫ��-����0-����-����1-ƫ��-����2-����-...-����-����7-ƫ��-β
	for(int i=0;i<NumOfHinge;i++)
	{
		BodyJoint[i] = dJointCreateHinge(world,0);
		dJointAttach(BodyJoint[i],SnakeBody[i].body,SnakeBody[i+1].body);	//	���ӵ�i��ģ��͵�i+1��ģ��
		dJointSetHingeAnchor(BodyJoint[i],-BodyLength-BodyLength*2*i,0+deltaY,ModuleHeightCen);
		if(i%2 == 0)
			dJointSetHingeAxis(BodyJoint[i],0,1,0);	//��Y����Ϊż�������ؽڵ���ת�᣺�������ɶ�
		else
			dJointSetHingeAxis(BodyJoint[i],0,0,1);	//��Z����Ϊ���������ؽڵ���ת�᣺ƫ�����ɶ�
		dJointSetHingeParam(BodyJoint[i], dParamLoStop, -M_PI/2);
		dJointSetHingeParam(BodyJoint[i], dParamHiStop, M_PI/2);

	}
	
	//���ͷβģ��ƫ���ؽ�
	headJoint = dJointCreateHinge(world,0);
	tailJoint = dJointCreateHinge(world,0);
	dJointAttach(headJoint,SnakeHead.body,SnakeBody[0].body);	//	���ӵ�1��ģ���ͷģ��
	dJointAttach(tailJoint,SnakeBody[NumOfModule-1].body,SnakeTail.body);	//	���ӵ�8��ģ���βģ��
	dJointSetHingeAnchor(headJoint,0,0+deltaY,ModuleHeightCen);
	dJointSetHingeAnchor(tailJoint,-ModuleLength-BodyLength*2*NumOfHinge,0+deltaY,ModuleHeightCen);
	dJointSetHingeAxis(headJoint,0,0,1);	//��Z����Ϊ�����ؽڵ���ת��
	dJointSetHingeAxis(tailJoint,0,0,1);	//��Z����Ϊ�����ؽڵ���ת��
	dJointSetHingeParam(headJoint, dParamLoStop, -M_PI/2);
	dJointSetHingeParam(headJoint, dParamHiStop, M_PI/2);
	dJointSetHingeParam(tailJoint, dParamLoStop, -M_PI/2);
	dJointSetHingeParam(tailJoint, dParamHiStop, M_PI/2);


	
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
      contact[i].surface.mode = dContactBounce;//
      contact[i].surface.mu   = 0.8;
	  //contact[i].surface.mu2 = 0;//
      contact[i].surface.bounce     = 0.2; // (0.0~1.0) restitution parameter
      //contact[i].surface.bounce_vel = 0; // minimum incoming velocity for bounce
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
    }
  }
}

static void start()
{
  static float xyz[3] = {0,10,10};
  static float hpr[3] = {-90, -30, 0};
  dsSetViewpoint (xyz,hpr);				// �趨�ӵ������
}

//ѭ������ÿһ����ִ��һ��

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
	dReal sides[3] = {BodyLength,ModuleWidth,ModuleHeight};
	//dReal WallSize[3] = {WallLength,WallWidth,WallHeight};
	for(i=0;i<NumOfModule;i++)
	{
		dsSetColorAlpha(1,1,1,1);	//����ģ�����ɫ
		spos[i] = dBodyGetPosition(SnakeBody[i].body);		//���ģ�鵱ǰ��λ��
		sR[i]   = dBodyGetRotation(SnakeBody[i].body);		//���ģ�鵱ǰ����̬
		dReal SidesEven[3] = {BodyX,BodyY,BodyZ};
		//dReal SidesOdd[3] = {BodyY,BodyX,BodyZ};
		if(i%2==0)
			dsDrawBox(spos[i],sR[i],SidesEven);	//�û�õ�λ�˺ͳ��ȡ��뾶�Ȼ��Ƶ�ǰ��ģ��
		else
			dsDrawBox(spos[i],sR[i],SidesEven);	//�û�õ�λ�˺ͳ��ȡ��뾶�Ȼ��Ƶ�ǰ��ģ��
	}

	for(i=0;i<NumOfHeadWheels;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//����ģ�����ɫ
		spos[i] = dBodyGetPosition(HeadWheelL[i].body);		//���ģ�鵱ǰ��λ��
		sR[i]   = dBodyGetRotation(HeadWheelL[i].body);		//���ģ�鵱ǰ����̬
		dsDrawCylinder(spos[i],sR[i],WheelLength,WheelRadius);	//�û�õ�λ�˺ͳ��ȡ��뾶�Ȼ��Ƶ�ǰ��ģ��
	}

	for(i=0;i<NumOfHeadWheels;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//����ģ�����ɫ
		spos[i] = dBodyGetPosition(HeadWheelR[i].body);		//���ģ�鵱ǰ��λ��
		sR[i]   = dBodyGetRotation(HeadWheelR[i].body);		//���ģ�鵱ǰ����̬
		dsDrawCylinder(spos[i],sR[i],WheelLength,WheelRadius);	//�û�õ�λ�˺ͳ��ȡ��뾶�Ȼ��Ƶ�ǰ��ģ��
	}

	for(i=0;i<NumOfHeadWheels;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//����ģ�����ɫ
		spos[i] = dBodyGetPosition(TailWheelL[i].body);		//���ģ�鵱ǰ��λ��
		sR[i]   = dBodyGetRotation(TailWheelL[i].body);		//���ģ�鵱ǰ����̬
		dsDrawCylinder(spos[i],sR[i],WheelLength,WheelRadius);	//�û�õ�λ�˺ͳ��ȡ��뾶�Ȼ��Ƶ�ǰ��ģ��
	}

	for(i=0;i<NumOfHeadWheels;i++)
	{
		dsSetColorAlpha(1,0,0,1);	//����ģ�����ɫ
		spos[i] = dBodyGetPosition(TailWheelR[i].body);		//���ģ�鵱ǰ��λ��
		sR[i]   = dBodyGetRotation(TailWheelR[i].body);		//���ģ�鵱ǰ����̬
		dsDrawCylinder(spos[i],sR[i],WheelLength,WheelRadius);	//�û�õ�λ�˺ͳ��ȡ��뾶�Ȼ��Ƶ�ǰ��ģ��
	}

	dReal sidess[3] = {ModuleWidth,ModuleWidth,ModuleHeight};
	dsSetColorAlpha(1,1,1,0.8);	//����ͷβģ�����ɫ
	ap = dBodyGetPosition(SnakeHead.body);		//���ģ�鵱ǰ��λ��
	ar = dBodyGetRotation(SnakeHead.body);		//���ģ�鵱ǰ����̬
	dsDrawBox(ap,ar,sidess);	//�û�õ�λ�˺ͳ��ȡ��뾶�Ȼ��Ƶ�ǰ��ģ��
	ap = dBodyGetPosition(SnakeTail.body);		//���ģ�鵱ǰ��λ��
	ar = dBodyGetRotation(SnakeTail.body);		//���ģ�鵱ǰ����̬
	dsDrawBox(ap,ar,sidess);	//�û�õ�λ�˺ͳ��ȡ��뾶�Ȼ��Ƶ�ǰ��ģ��
	

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
		}// wǰ����s���ˣ�a��ת��d��ת
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
	

	world = dWorldCreate();//���ɶ���ѧ�����õ�world
	space = dHashSpaceCreate(0);//������ײ����õ�space
	contactgroup = dJointGroupCreate(0);//������ײ����õ�����ؽ�
	dWorldSetGravity(world,0,0,-9.81);//��Ϊ�����������ٶ�
	ground = dCreatePlane(space,0,0,1,0);//��space�д�����z=0ƽ�棬������ײ���

	dWorldSetERP(world,0.2);//����ERP����
	dWorldSetCFM(world,1e-5);//����CFM����
	
	headTailCreator();
	bodyCreator();
	jointConnect();

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
	float desire_angle[NumOfHinge+2] = {};	//Ŀ��Ƕ�{BodyJoint[0],...,BodyJoint[NumOfHinge-1],headJoint,tailJoint}
	float diff_angle[NumOfHinge+2] = {};		//��ǰ�Ƕ�-Ŀ��Ƕ�
	float maxtorque = 1000;

	if(steps<4000)
	{
		//�����������߹켣
		for(int i=0; i<NumOfHinge; i++)
		{
				desire_angle[i] = 0;
		}
		desire_angle[NumOfHinge] = 0;//ͷ��ƫ���Ƕ� M_PI/18000*steps;
		desire_angle[NumOfHinge+1] = 0;//β��ƫ���Ƕ�
		int HLvel = 0;
		int HRvel = 0;
		int TLvel = 0;
		int TRvel = 0;

		switch (ch)
		{//119w,115s,97a,100d
		case 119:
			HLvel = -100;
			HRvel = -100;
			TLvel = -100;
			TRvel = -100;
			break;
		case 115:
			HLvel = 100;
			HRvel = 100;
			TLvel = 100;
			TRvel = 100;
			break;
		case 97:
			HLvel = -100;
			HRvel = -200;
			TLvel = -100;
			TRvel = -100;
			break;
		case 100:
			HLvel = -200;
			HRvel = -100;
			TLvel = -100;
			TRvel = -100;
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
		for(int i=0; i<NumOfHinge; i++)
		{
			desire_angle[i] = 0;
		}
		desire_angle[NumOfHinge] = 0;
		desire_angle[NumOfHinge+1] = 0;
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
