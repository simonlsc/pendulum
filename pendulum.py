from manim import *
from numpy import sin, cos
import numpy as np
import scipy.integrate as integrate

G = 9.8  # acceleration due to gravity, in m/s^2

# create a time array with dt steps, global for all Scenes in this file
duration=58  # in seconds

dt = 0.033 # 30 frame per second
t = np.arange(0, duration, dt) # for odeint
oneSixth=int(np.floor(len(t)/6))
#stick=np.concatenate((np.ones(oneSixth),np.linspace(1.0,0.0,oneSixth),np.zeros(len(t)-2*oneSixth)))
stick=np.concatenate((np.ones(oneSixth),np.linspace(1.0,0.3,oneSixth),np.linspace(0.3,0.1,len(t)-2*oneSixth)))
stick=np.ones(len(t))

# simple pendulum model
def derivs(state, t, l, w):
    dydx = np.zeros_like(state)
    dydx[0] = state[1]
    dydx[1] = - G * sin(state[0])/ l
    return dydx

def getline(Point1: Mobject,Point2: Mobject) -> Line:
    return Line(Point1.get_center(),Point2.get_center()).set_stroke(width=2)

class SimplePendulum(Dot):
    def __init__(self, length, weight, theta0, w0, colour):
        super().__init__()                
        self.localClock=0
        self.length=length
        self.weight=weight

        self.state = np.radians([theta0,w0])
        p=(length,weight)
        self.y = integrate.odeint(derivs, self.state, t, p) # solve ode (ordinary differential equation)
        self.x1 =  length*sin(self.y[:, 0])
        self.y1 = -length*cos(self.y[:, 0])
        self.mass = Dot(radius=0.04*weight).move_to(self.get_x()+self.x1[0]*RIGHT+self.get_y()+self.y1[0]*UP).set_color(colour)
        self.line = getline(self,self.mass)
        self.line.add_updater(lambda mob: mob.become(getline(self,self.mass).set_opacity(stick[self.getLocalTime()])))
        self.set_opacity(0).set_color(colour)

        self.traj = VGroup()
        self.trajLength=20
        self.dimBasis=1/self.trajLength

        """A SimplePendulum
        Parameters
        ----------
        length
            Length of pendulum.
        weight
            Weight  of pendulum.
        theta0
            Initial angle.
        w0
            Initial anglar velocity
        colour
            Colour of the weight.
        """
        
    def clockTick(self):
        self.localClock += 1

    def getLocalTime(self):
        return self.localClock
    
    
def trajectoryRefresh(sp: SimplePendulum):
    sp.traj = VGroup()
    i = sp.localClock
    a = sp.trajLength
    if sp.localClock >= sp.trajLength:
        for n in range(i,i-sp.trajLength,-1):            
            sp.traj.add(Line(
                            sp.get_center() + sp.x1[n]*RIGHT + sp.y1[n]*UP , 
                            sp.get_center() + sp.x1[n-1]*RIGHT + sp.y1[n-1]*UP)
                            .set_stroke(sp.get_color(),width=2)
                            .set_opacity(sp.dimBasis*a))
            a -= 1
    else:
        for n in range(i,0,-1):
            sp.traj.add(Line(
                            sp.get_center() + sp.x1[n]*RIGHT + sp.y1[n]*UP , 
                            sp.get_center() + sp.x1[n-1]*RIGHT + sp.y1[n-1]*UP)
                            .set_stroke(sp.get_color(),width=2)
                            .set_opacity(sp.dimBasis*a))
            a -= 1

# DoublePendulum is made of 2 Simple Pendulums
# with a completely different differential equations
class DoublePendulum():
    def __init__(self,SP1: SimplePendulum,SP2: SimplePendulum):

        self.sp1 = SP1
        self.sp2 = SP2

        self.state = np.concatenate((SP1.state,SP2.state))

        def doublePendulumModel(state, t, l1, m1, l2, m2):
            dydx = np.zeros_like(state)
            dydx[0] = state[1]

            delta = state[2] - state[0]
            denominator1 = (m1 + m2) * l1 - m2 * l1 * cos(delta) * cos(delta)
            dydx[1] = ((m2 * l1 * state[1] * state[1] * sin(delta) * cos(delta)
                        + m2 * G * sin(state[2]) * cos(delta)
                        + m2 * l2 * state[3] * state[3] * sin(delta)
                        - (m1+m2) * G * sin(state[0]))
                       / denominator1)

            dydx[2] = state[3]

            denominator2 = (l2/l1) * denominator1
            dydx[3] = ((- m2 * l2 * state[3] * state[3] * sin(delta) * cos(delta)
                        + (m1+m2) * G * sin(state[0]) * cos(delta)
                        - (m1+m2) * l1 * state[1] * state[1] * sin(delta)
                        - (m1+m2) * G * sin(state[2]))
                        / denominator2)

            return dydx

        p=(SP1.length,SP1.weight,SP2.length,SP2.weight)
        self.y = integrate.odeint(doublePendulumModel, self.state, t, p) # solve ode (ordinary differential equation)
        SP1.x1 =  SP1.length*sin(self.y[:, 0])
        SP1.y1 = -SP1.length*cos(self.y[:, 0])
        # sp2.x1,y1 are reference to fix point of sp1
        SP2.x1 =  SP2.length*sin(self.y[:, 2]) + SP1.x1
        SP2.y1 = -SP2.length*cos(self.y[:, 2]) + SP1.y1
        # move SP2.mass to initial position
        SP2.mass.move_to(np.array([SP2.x1[0],SP2.y1[0],0]))
        SP2.line.clear_updaters()
        SP2.line.add_updater(lambda mob: mob.become(getline(SP1.mass,SP2.mass).set_opacity(stick[SP2.getLocalTime()])))

        self.localClock=0
        self.trajLength=20
        self.dimBasis=1/self.trajLength
        
    def clockTick(self):
        self.localClock += 1




class Pendulum(Scene):
    def construct(self):
        axes = NumberPlane().set_opacity(0.2)
        self.add(axes)

        Text1 = MathTex(r"""
                m &=3 \text{ kg} \\
                l &= 2 \text{ m} \\
                \theta_1 &= 170^o
                """).to_edge(UP+LEFT)

#        self.add(Text1)

        L = 2.0  # length of pendulum 1 in m
        M = 3.0  # mass of pendulum 1 in kg

        # th is the initial angle (degrees)
        # w is the initial angular velocity (degrees per second)
        th = 170.0
        w = 0.0
        sp = SimplePendulum(L,M,th,w,YELLOW)

        self.add(sp, sp.line, sp.mass)

        def trajectoryUpdate(sp: SimplePendulum):
            self.remove(sp.traj)
            trajectoryRefresh(sp)
            self.add(sp.traj)    

        for i in range(len(t)-1): # iterate through differential equation results
            trajectoryUpdate(sp)
            sp.clockTick()
            sp.mass.generate_target()
            sp.mass.target.move_to(sp.get_center() + sp.x1[i+1]*RIGHT + sp.y1[i+1]*UP)
            self.play(MoveToTarget(sp.mass),run_time=dt,rate_func=linear)

    def getline(self,Point1: Mobject, Point2: Mobject):
            start_point = Point1.get_center()
            end_point = Point2.get_center()
            line = Line(start_point,end_point).set_stroke(width=2) 
            return line

class TwoPendulums(Scene):
    def construct(self):
        axes = NumberPlane().set_opacity(0.2)
        self.add(axes)

        Text1 = MathTex(r"""
                m_1 &=3 \text{ kg} \\
                l_1 &= 2 \text{ m} \\
                \theta_1 &= 170^{\circ}
                """).to_edge(UP+LEFT).set_color(YELLOW)

        self.add(Text1)

        Text2 = MathTex(r"""
                m_2 &=4 \text{ kg} \\
                l_2 &= 1.5 \text{ m} \\
                \theta_2 &= 90^{\circ}
                """).to_edge(UP+RIGHT).set_color(BLUE)

        self.add(Text2)

        L1 = 2.0  # length of pendulum 1 in m
        L2 = 1.5  # length of pendulum 2 in m
        M1 = 3.0  # mass of pendulum 1 in kg
        M2 = 4.0  # mass of pendulum 2 in kg

        th1 = 170.0
        w1 = 0.0
        th2 = 90.0
        w2 = 0.0

        dp = DoublePendulum(SimplePendulum(L1,M1,th1,w1,YELLOW),SimplePendulum(L2,M2,th2,w2,BLUE))

        self.add(dp.sp1,dp.sp1.line,dp.sp1.mass)
        self.add(dp.sp2,dp.sp2.line,dp.sp2.mass)

        def trajectoryUpdate(sp: SimplePendulum):
            self.remove(sp.traj)
            trajectoryRefresh(sp)
            self.add(sp.traj)    

#        trace = TracedPath(dp.sp2.mass.get_center,stroke_width=1,stroke_color=BLUE, dissipating_time=1)
#        self.add(trace)
        for i in range(len(t)-1): # iterate through differential equation results
            trajectoryUpdate(dp.sp2)
            dp.sp2.clockTick()
            dp.clockTick()
            dp.sp1.mass.generate_target()
            dp.sp2.mass.generate_target()
            dp.sp1.mass.target.move_to(dp.sp1.get_center() + dp.sp1.x1[i+1]*RIGHT + dp.sp1.y1[i+1]*UP)
            # sp2.x1,y1 are reference to fix point of sp1
            dp.sp2.mass.target.move_to(dp.sp1.get_center() + dp.sp2.x1[i+1]*RIGHT + dp.sp2.y1[i+1]*UP)
            self.play(MoveToTarget(dp.sp1.mass),MoveToTarget(dp.sp2.mass), run_time=dt,rate_func=linear)

class TitleDoublePendulum(Scene):
    def construct(self):
        title = MathTex("Double\ Pendulum").scale(3)
        self.play(Write(title))
        self.wait(3)
        self.play(FadeOut(title))


class ThreeDoublePendulums(Scene):
    def construct(self):
        axes = NumberPlane().set_opacity(0.2)
        self.add(axes)

        self.add(MathTex("Initial\ Condition\ of\ the\ 3\ Double\ Pendulums\ are\ differed\ by\ 0.001^{\circ}").to_edge(UP).scale(0.7))

        L1 = 2.0  # length of pendulum 1 in m
        L2 = 1.5  # length of pendulum 2 in m
        M1 = 3.0  # mass of pendulum 1 in kg
        M2 = 4.0  # mass of pendulum 2 in kg

        th1 = 170.0
        w1 = 0.0
        th2 = 90.0
        w2 = 0.0

        epsilon=0.001 # degree

        dpList = []
        dpList.append(DoublePendulum(SimplePendulum(L1,M1,th1,w1,RED),SimplePendulum(L2,M2,th2-epsilon,w2,PINK)))
        dpList.append(DoublePendulum(SimplePendulum(L1,M1,th1,w1,GREEN),SimplePendulum(L2,M2,th2,w2,YELLOW)))
        dpList.append(DoublePendulum(SimplePendulum(L1,M1,th1,w1,BLUE),SimplePendulum(L2,M2,th2+epsilon,w2,PURPLE)))

        for dp in dpList:
            dp.sp1.mass.set_opacity(0.75)
            dp.sp2.mass.set_opacity(0.75)
            self.add(dp.sp1,dp.sp1.line,dp.sp1.mass)
            self.add(dp.sp2,dp.sp2.line,dp.sp2.mass)

        def trajectoryUpdate(sp: SimplePendulum):
            self.remove(sp.traj)
            trajectoryRefresh(sp)
            self.add(sp.traj)    

        trace1 = TracedPath(dpList[0].sp2.mass.get_center,stroke_width=1,stroke_color=dpList[0].sp2.get_color())
        trace2 = TracedPath(dpList[1].sp2.mass.get_center,stroke_width=1,stroke_color=dpList[1].sp2.get_color())
        trace3 = TracedPath(dpList[2].sp2.mass.get_center,stroke_width=1,stroke_color=dpList[2].sp2.get_color())
#        self.add(trace)

        # for fading out components per give time line
        mass1=np.concatenate((np.ones(2*oneSixth),np.linspace(1.0,0.0,oneSixth),np.zeros(len(t)-3*oneSixth)))
        mass2=np.concatenate((np.ones(3*oneSixth),np.linspace(1.0,0.0,oneSixth),np.zeros(len(t)-4*oneSixth)))

        for i in range(len(t)-1): # iterate through differential equation results
            # turn on tracing after 1/2 the total time
            if i == int(len(t)/2):
                self.add(trace1,trace2,trace3)
            animationList=[]
            for dp in dpList:
                dp.sp1.line.set_opacity(stick[dp.localClock])
                dp.sp2.line.set_opacity(stick[dp.localClock])
                dp.sp1.mass.set_opacity(mass1[dp.localClock])
                dp.sp2.mass.set_opacity(mass2[dp.localClock])

                trajectoryUpdate(dp.sp2)
                dp.sp1.clockTick()
                dp.sp2.clockTick()
                dp.clockTick()
                dp.sp1.mass.generate_target()
                dp.sp2.mass.generate_target()
                dp.sp1.mass.target.move_to(dp.sp1.get_center() + dp.sp1.x1[i+1]*RIGHT + dp.sp1.y1[i+1]*UP)
                # sp2.x1,y1 are reference to fix point of sp1
                dp.sp2.mass.target.move_to(dp.sp1.get_center() + dp.sp2.x1[i+1]*RIGHT + dp.sp2.y1[i+1]*UP)
                animationList.append(MoveToTarget(dp.sp1.mass))
                animationList.append(MoveToTarget(dp.sp2.mass))
            self.play(*animationList, run_time=dt,rate_func=linear)

class TitleThreeDoublePendulum(Scene):
    def construct(self):
        title = MathTex("3\\times Double\ Pendulums").scale(2)
        self.play(Write(title))
        self.wait(2)
        self.play(FadeOut(title))

class SevenSP(Scene):
    def construct(self):
        axes = NumberPlane().set_opacity(0.2)
        self.add(axes)

        def trajectoryUpdate(sp: SimplePendulum):
            self.remove(sp.traj)
            trajectoryRefresh(sp)
            self.add(sp.traj)                
                            
        #Pendulums Motions
        lengthList = np.arange(0.5,7,0.25)
        weightList = [3.0]*len(lengthList)
        theta0List = [20.0]*len(lengthList)
        w0List = [0]*len(lengthList)
        
        colourList = color_gradient([PURE_RED,YELLOW,PURE_GREEN,PURE_BLUE,PURPLE],len(lengthList))

        pGroup = VGroup(*list(map(lambda l,w,t,w0,c: SimplePendulum(l,w,t,w0,c),lengthList,weightList,theta0List,w0List,colourList))).shift(UP*3)
        self.play(FadeIn(pGroup))
        for p in pGroup.submobjects:
            self.add(p.line)

        for i in range(len(t)-1): # iterate through differential equation results
            animations = []
            for p in pGroup.submobjects:
                trajectoryUpdate(p)
                p.clockTick()
                p.mass.generate_target()
                p.mass.target.move_to(p.get_center() + p.x1[i+1]*RIGHT + p.y1[i+1]*UP)
                animations.append(MoveToTarget(p.mass))
            self.play(*animations,run_time=dt,rate_func=linear)


class weightPendulums(Scene):
    def construct(self):
        axes = NumberPlane().set_opacity(0.2)
        self.add(axes)

        title=MathTex("Pendulum\ period\ is\ independent\ with\ weight,\ T=2 \pi \sqrt{\\frac{L}{g}}")
        #\ T=2 \pi \sqrt{\frac{L}{g}}")
        title.to_edge(UP).scale(0.7)

        self.play(FadeIn(title))    

        def trajectoryUpdate(sp: SimplePendulum):
            self.remove(sp.traj)
            trajectoryRefresh(sp)
            self.add(sp.traj)                
                            
        #Pendulums Motions
        weightList = np.arange(0.5,6.0,0.5)
        lengthList = [3.0]*len(weightList)
        theta0List = [30.0]*len(lengthList)
        w0List = [0]*len(lengthList)
        
        colourList = color_gradient([PURE_RED,YELLOW,PURE_GREEN,PURE_BLUE,PURPLE],len(lengthList))

        pGroup = VGroup(*list(map(lambda l,w,t,w0,c: SimplePendulum(l,w,t,w0,c),lengthList,weightList,theta0List,w0List,colourList))).arrange(buff=1,direction=RIGHT)
        self.add(pGroup.shift(UP*2))
        for p in pGroup.submobjects:
            self.add(p.line)

        for i in range(len(t)-1): # iterate through differential equation results
            animations = []
            for p in pGroup.submobjects:
                trajectoryUpdate(p)
                p.clockTick()
                p.mass.generate_target()
                p.mass.target.move_to(p.get_center() + p.x1[i+1]*RIGHT + p.y1[i+1]*UP)
                animations.append(MoveToTarget(p.mass))
            self.play(*animations,run_time=dt,rate_func=linear)

# class DynamicPendulum(SimplePendulum):
#       add always_update to allow dynamic change of lenght, weight and the fix point Dot