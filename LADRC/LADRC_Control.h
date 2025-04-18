//
// Created by 32806 on 24-9-23.
//

#ifndef LADRC_CONTROL_H
#define LADRC_CONTROL_H

/**
   *@Brief default 参数表
   *@Brief 根据经验 ts在0.2~0.3之间 Wo与Wc选定后从小到大整定b0
   *@Date@WangShun 2022-05-28 2022-07-03补充
---------------------------------------------------
		      LADRC default参数表									
---------------------------------------------------
---------------------------------------------------
	ts	|	h	|	r	|   wc   |   w0  |	b0
---------------------------------------------------
	0.1	|	h	|	r	|  100   |  400  |	b0
---------------------------------------------------
   0.157|	h	|	r	|   64   |  224~255  |	b0
---------------------------------------------------
   0.158|	h	|	r	|   63   |  253  |	b0
---------------------------------------------------
   0.159|	h	|	r	|   63   |  252  |	b0
---------------------------------------------------
	0.16|	h	|	r	|   63   |  250  |	b0
---------------------------------------------------
	0.17|	h	|	r	|   59   |  235  |	b0
---------------------------------------------------
	0.18|	h	|	r	|   56   |  222  |	b0
---------------------------------------------------
	0.2	|	h	|	r	|   50   |  200  |	b0
---------------------------------------------------
	0.21|	h	|	r	|   48   |  190  |	b0
---------------------------------------------------
	0.22|	h	|	r	|   45   |  182  |	b0
---------------------------------------------------
	0.23|	h	|	r	|   43   |  174  |	b0
---------------------------------------------------
	0.24|	h	|	r	|   42   |  167  |	b0
---------------------------------------------------
	0.25|	h	|	r	|   40   |  160  |	b0
---------------------------------------------------
	0.26|	h	|	r	|   38   |  154  |	b0
---------------------------------------------------
	0.27|	h	|	r	|   37   |  148 |	b0
---------------------------------------------------
	0.28|	h	|	r	|   36   |  144  |	b0
---------------------------------------------------
	0.29|	h	|	r	|   34   |  138  |	b0
---------------------------------------------------
	0.3	|	h	|	r	|   33   |  133  |	b0
---------------------------------------------------
	0.4	|	h	|	r	|   25   |  100  |	b0
---------------------------------------------------
	0.5	|	h	|	r	|   20   |   80  |	b0
---------------------------------------------------
---------------------------------------------------
*/

class LADRC_Control {

private:
    float v1,v2;         //最速输出值
    float r;             //速度因子
    float h;             //积分步长
    float z1,z2,z3;      //观测器输出
    float w0,wc,b0,u;    //观测器带宽 控制器带宽 系统参数 控制器输出

public:
    LADRC_Control();
    LADRC_Control(float LADRC_r,float LADRC_h,float LADRC_w0,float LADRC_wc,float LADRC_b0);
    ~LADRC_Control();

    //成员函数
    void LADRC_DeInit(float LADRC_v1,float LADRC_v2,float LADRC_r,float LADRC_h,float LADRC_wc,float LADRC_b0);//重定义参数
    void LADRC_TD(float Expect);//LADRC跟踪微分部分，期望值Expect(v0)
    void LADRC_ESO(float FeedBack);//LADRC线性状态观测器
    void LADRC_LF();//线性控制率
    void LADRC_Loop(float* Expect,float* RealTimeOut);//LADRC控制函数,实际输出RealTimeOut

};



#endif //LADRC_CONTROL_H
