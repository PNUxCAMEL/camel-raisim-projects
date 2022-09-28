//
// Created by hs on 22. 8. 16.
//

#include "qpsolver.h"

#define BIG_NUMBER 5e10
#define GRAVITY -9.81
#define BODYMASS 10
#define LHIP 0.085
#define LTHI 0.2
#define LCAL 0.2

#define F_MAX 240
#define MU 0.6

char var_elim[2000];
char con_elim[2000];

ConvexMPCSolver::~ConvexMPCSolver() {
    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);

    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
}

/**
 * Set the parameters for MPC controller by qt GUI
 * @param Horizon : MPC Horizon -> init : 5
 * @param Dt      : MPC Contoller time step -> init : 0.005s = 5ms
 */
void ConvexMPCSolver::setParameters(int Horizon, double Dt){
    if(_Horizon != Horizon)
    {
        _Horizon = Horizon;
        resizeMatrix();
    }
    _Dt = Dt;
}

/**
 * Set the wieghts for MPC controller by qt GUI
 * @param weightMat : Weights for robot states 12[Q,P,dQ,dP] + gravity
 * @param alpha     : Weights for desired force U
 */
void ConvexMPCSolver::setWeights(Vec13<double> weightMat, double alpha){
    _weightMat = weightMat;
    _alpha = alpha;
}

void ConvexMPCSolver::resizeMatrix(){
    Aqp.resize(13*_Horizon, Eigen::NoChange);
    Bqp.resize(13*_Horizon, 12*_Horizon);
    L.resize(13*_Horizon, 13*_Horizon);
    xd.resize(13*_Horizon, Eigen::NoChange);
    U_b.resize(20*_Horizon, Eigen::NoChange);
    fmat.resize(20*_Horizon, 12*_Horizon);
    H.resize(12*_Horizon, 12*_Horizon);
    g.resize(12*_Horizon, Eigen::NoChange);
    K.resize(12*_Horizon, 12*_Horizon);

    Aqp.setZero();
    Bqp.setZero();
    L.setZero();
    xd.setZero();
    U_b.setZero();
    fmat.setZero();
    H.setZero();
    K.setIdentity();

    if(real_allocated)
    {
        free(H_qpoases);
        free(g_qpoases);
        free(A_qpoases);
        free(lb_qpoases);
        free(ub_qpoases);
        free(q_soln);

        free(H_red);
        free(g_red);
        free(A_red);
        free(lb_red);
        free(ub_red);
        free(q_red);
    }

    H_qpoases = (qpOASES::real_t*)malloc(12*_Horizon*12*_Horizon*sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t*)malloc(12*_Horizon*sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t*)malloc(12*_Horizon*20*_Horizon*sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t*)malloc(20*_Horizon*sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t*)malloc(20*_Horizon*sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t*)malloc(12*_Horizon*sizeof(qpOASES::real_t));

    H_red = (qpOASES::real_t*)malloc(12*12*_Horizon*_Horizon*sizeof(qpOASES::real_t));
    g_red = (qpOASES::real_t*)malloc(12*1*_Horizon*sizeof(qpOASES::real_t));
    A_red = (qpOASES::real_t*)malloc(12*20*_Horizon*_Horizon*sizeof(qpOASES::real_t));
    lb_red = (qpOASES::real_t*)malloc(20*1*_Horizon*sizeof(qpOASES::real_t));
    ub_red = (qpOASES::real_t*)malloc(20*1*_Horizon*sizeof(qpOASES::real_t));
    q_red = (qpOASES::real_t*)malloc(12*_Horizon*sizeof(qpOASES::real_t));
    real_allocated = 1;
}

void ConvexMPCSolver::matrixinitialize(raisim::Mat<3,3> bdyInertia){
    x0.setZero();
    xd.setZero();
    K.setIdentity();
    Aqp.setZero();
    Bqp.setZero();
    L.setZero();
    H.setZero();
    g.setZero();
    U_b.setZero();
    fmat.setZero();
    I_world = bdyInertia.e();
}

void ConvexMPCSolver::setTrajectory(double currentTime,GaitType currentGait) {
    for(int i = 0; i < _Horizon ; i++)
    {
        xd(i*13+5,0) = 0.37;

        if(currentGait == GaitType::STAND)
        {
            xd(i*13+3,0) = stopPosX;
            xd(i*13+4,0) = stopPosY;
            xd(i*13+9,0) = 0.0;
            xd(i*13+10,0) = 0.0;
        }
        else
        {
            xd(i*13+3,0) = p[0]+mdesiredVx*(_Dt*i);
            xd(i*13+4,0) = p[1]+mdesiredVy*(_Dt*i);
            xd(i*13+9,0) = mdesiredVx;
            xd(i*13+10,0) = mdesiredVy;
        }
    }

    desiredPositionX = mdesiredVx;
    desiredPositionY = xd(4,0);
    desiredPositionZ = xd(5,0);

    desiredRotationX = xd(0,0);
    desiredRotationY = xd(1,0);
    desiredRotationZ = xd(2,0);
}

void ConvexMPCSolver::getMetrices(int *_mpcTable, raisim::VecDyn pos, raisim::VecDyn vel, raisim::Vec<3> footPosition[4]){
    p << pos[0], pos[1], pos[2];
    quat << pos[3], pos[4], pos[5], pos[6];
    v << vel[0], vel[1], vel[2];
    w << vel[3], vel[4], vel[5];

    quat_to_euler(quat, q);
    x0 << q[0], q[1], q[2], p[0], p[1], p[2], w[0], w[1], w[2], v[0], v[1], v[2], GRAVITY;

    ss_mats(Ac, Bc, footPosition);
    c2qp(Ac,Bc);
    L.diagonal() = _weightMat.replicate(_Horizon,1);

    H = 2*(Bqp.transpose()*L*Bqp + _alpha*K);
    g = 2*Bqp.transpose()*L*(Aqp*x0 - xd);

    int k = 0;
    for(int i = 0; i < _Horizon; i++){
        for(int16_t j = 0; j < 4; j++){
            U_b(5*k + 0) = BIG_NUMBER;
            U_b(5*k + 1) = BIG_NUMBER;
            U_b(5*k + 2) = BIG_NUMBER;
            U_b(5*k + 3) = BIG_NUMBER;
            U_b(5*k + 4) = F_MAX*_mpcTable[i*4+j];
            k++;
        }
    }
    //fmat initialize
    float mu = 1.f/MU;
    Eigen::Matrix<double,5,3> f_block;

    f_block <<  mu, 0,  1.f,
            -mu, 0,  1.f,
            0,  mu, 1.f,
            0, -mu, 1.f,
            0,   0, 1.f;

    for(int i = 0; i < _Horizon*4; i++) //12
    {
        fmat.block(i*5,i*3,5,3) = f_block; //Such like diagonal matrix
    }
}

int8_t near_zero(float a)
{
    return (a < 0.01 && a > -.01) ;
}

int8_t near_one(float a)
{
    return near_zero(a-1);
}

void ConvexMPCSolver::matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols){
    int32_t a = 0;
    for(int16_t r = 0; r < rows; r++){
        for(int16_t c = 0; c < cols; c++){
            dst[a] = src(r,c);
            a++;
        }
    }
}

void ConvexMPCSolver::qpSolver(){
    matrix_to_real(H_qpoases, H, 12*_Horizon, 12*_Horizon);
    matrix_to_real(g_qpoases, g, 12*_Horizon, 1);
    matrix_to_real(A_qpoases, fmat, 20*_Horizon, 12*_Horizon);
    matrix_to_real(ub_qpoases,U_b, 20*_Horizon, 1);

    for(int i =0; i<20*_Horizon; i++)
        lb_qpoases[i] = 0.f;

    // Set red matrices
    int16_t num_constraints = 20*_Horizon;
    int16_t num_variables = 12*_Horizon;

    int new_cons = num_constraints;
    int new_vars = num_variables;

    //Set the length of metrices depend on variables and constraintss
    for(int i=0; i<num_constraints; i++)
        con_elim[i] = 0;
    for(int i=0; i<num_variables; i++)
        var_elim[i] = 0;

    // Both of lb, ub is not near the zero, var_elim[5]/con_elim[3] => 1
    // the other side, var_elim[5]/con_elim[3] => 0
    for(int i=0; i<num_constraints; i++)
    {
        // near the zero (-0.01 ~ 0.01)
        // lb_qpoases => all zero
        // ub_qpoases => 0~3: BIG_VALUE ,
        //            =>  4 : depend on gait[0 or 1]*f_max / if stand it all set f_max.
        if(!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i])))
            continue;

        // Both of lb, ub is not near the zero
        // If gait is 0, the code below is passed
        double* c_row = &A_qpoases[i*num_variables];
        for(int j=0; j<num_variables; j++)
        {
            if(near_one(c_row[j])) //f_block third column values
            {
                new_vars -= 3;
                new_cons -= 5;
                int cs = (j*5)/3 -3;
                var_elim[j-2] = 1;
                var_elim[j-1] = 1;
                var_elim[j  ] = 1;
                con_elim[cs+4] = 1;
                con_elim[cs+3] = 1;
                con_elim[cs+2] = 1;
                con_elim[cs+1] = 1;
                con_elim[cs  ] = 1;
            }
        }
    }

    int var_idx[new_vars];
    int con_idx[new_cons];
    int count = 0;

    // If gait is 1, save the indexes
    for(int i=0; i<num_variables; i++)
    {
        // If gait is 1
        if(!var_elim[i])
        {
            if(!(count<new_vars))
            {
                std::cout << "BAD ERROR" << std::endl;
            }
            var_idx[count] = i;
            count++;
        }
    }
    count=0;
    for(int i=0; i<num_constraints; i++)
    {
        // If gait is 1
        if(!con_elim[i])
        {
            if(!(count<new_cons))
            {
                std::cout << "BAD ERROR" << std::endl;
            }
            con_idx[count] = i;
            count++;
        }
    }

    for(int i=0; i<new_vars;i++)
    {
        int old_a = var_idx[i];
        g_red[i] = g_qpoases[old_a];
        for(int j=0; j<new_vars; j++)
        {
            int old_b = var_idx[j];
            H_red[i*new_vars+j] = H_qpoases[old_a * num_variables + old_b];
        }
    }
    for(int i=0; i<new_cons;i++)
    {
        for(int j=0; j<new_vars; j++)
        {
            float cval = A_qpoases[(num_variables*con_idx[i]) + var_idx[j]];
            A_red[i*new_vars+j] = cval;
        }
    }

    for(int i=0; i<new_cons; i++)
    {
        int old = con_idx[i];
        ub_red[i] = ub_qpoases[old];
        lb_red[i] = lb_qpoases[old];
    }

    // Solve the problem using qpOASES
    qpOASES::int_t nWSR = 10000;
    qpOASES::QProblem problem(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem.setOptions(op);
    problem.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);

    int rval = problem.getPrimalSolution(q_red);
    if(rval != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");

    int vc = 0;
    for(int i = 0; i < num_variables; i++)
    {
        if(var_elim[i])
        {
            q_soln[i] = 0.0f;
        }
        else
        {
            q_soln[i] = q_red[vc];
            vc++;
        }
    }
}

template <class T>
T t_min(T a, T b)
{
    if(a<b) return a;
    return b;
}

template <class T>
T sq(T a)
{
    return a*a;
}

inline Eigen::Matrix<double,3,3> getSkew(Vec3<double> r)
{
    Eigen::Matrix3d cm;
    cm << 0.f, -r(2), r(1),
            r(2), 0.f, -r(0),
            -r(1), r(0), 0.f;
    return cm;
}

void ConvexMPCSolver::quat_to_euler(Vec4<double>& quat, Vec3<double>& q)
{
    //edge case!
    float as = t_min(-2.*(quat[1]*quat[3]-quat[0]*quat[2]),.99999);
    q(0) = atan2(2.f*(quat[2]*quat[3]+quat[0]*quat[1]),sq(quat[0]) - sq(quat[1]) - sq(quat[2]) + sq(quat[3]));
    q(1) = asin(as);
    q(2) = atan2(2.f*(quat[1]*quat[2]+quat[0]*quat[3]),sq(quat[0]) + sq(quat[1]) - sq(quat[2]) - sq(quat[3]));
}

void ConvexMPCSolver::ss_mats(Eigen::Matrix<double,13,13>& A, Eigen::Matrix<double,13,12>& B, raisim::Vec<3> footPosition[4]){
    double yc = cos(q[2]);
    double ys = sin(q[2]);

    R_yaw <<  yc,  -ys,   0,
            ys,  yc,   0,
            0,   0,   1;

    A.setZero();
    A(3,9) = 1.f;
    A(4,10) = 1.f;
    A(5,11) = 1.f;
    A(11,12) = 1.f;
    A.block(0,6,3,3) = R_yaw.transpose();

    //Get foot position on the body frame
    for(int i=0; i<4; i++) {
        for (int j = 0; j < 3; j++){
            footPosition[i][j] -= p[j];
        }
    }

    Eigen::Matrix<double,4,3> R_feet;
    R_feet <<  footPosition[0][0], footPosition[0][1], footPosition[0][2], //FR
            footPosition[1][0], footPosition[1][1], footPosition[1][2], //FL
            footPosition[2][0], footPosition[2][1], footPosition[2][2], //RR
            footPosition[3][0], footPosition[3][1], footPosition[3][2]; //RL
    B.setZero();

    I_world = R_yaw*I_world*R_yaw.transpose();
    I_inv = I_world.inverse();

    for(int n=0; n<4; n++)
    {
        B.block(6,n*3,3,3) = I_inv*getSkew(R_feet.row(n));
        B.block(9,n*3,3,3) = Eigen::Matrix3d::Identity() / BODYMASS;
    }
}

void ConvexMPCSolver::c2qp(Eigen::Matrix<double,13,13> A, Eigen::Matrix<double,13,12> B){
    ABc.setZero();
    ABc.block(0,0,13,13) = A;
    ABc.block(0,13,13,12) = B;
    ABc = _Dt*ABc;
    expmm = ABc.exp();

    Adt = expmm.block(0,0,13,13);
    Bdt = expmm.block(0,13,13,12);

    Eigen::Matrix<double,13,13> D[20];
    D[0].setIdentity();
    for(int i=1; i<=_Horizon; i++)
    {
        D[i] = Adt * D[i-1];
    }

    for(int r=0; r<_Horizon; r++)
    {
        Aqp.block(13*r,0,13,13) = D[r+1];
        for(int c=0; c<_Horizon; c++)
        {
            if(r>=c)
            {
                int a_num = r-c;
                Bqp.block(13*r,12*c,13,12) = D[a_num]*Bdt;
            }
        }
    }
}

void ConvexMPCSolver::getJacobian(Eigen::Matrix<double,3,3>& J, double hip, double thigh, double calf, int side){
    double s1 = std::sin(hip);
    double s2 = std::sin(thigh);

    double c1 = std::cos(hip);
    double c2 = std::cos(thigh);

    double s32 = std::sin(calf+thigh);
    double c32 = std::cos(calf+thigh);

    //right leg side = 1 / left leg side = -1
    J << 0,
        LTHI*c2+LCAL*c32,
        LCAL*c32,

        (-1)*side*LHIP*s1-LTHI*c1*c2-LCAL*c1*c32,
        LTHI*s1*s2+LCAL*s1*s32,
        LCAL*s1*s32,

        side*LHIP*c1-LTHI*s1*c2-LCAL*s1*c32,
        -LTHI*c1*s2-LCAL*c1*s32,
        -LCAL*c1*s32;
}

void ConvexMPCSolver::getGRF(Vec3<double> _f[4]){
    // [Fx,Fy,Fz]
    for(int leg = 0; leg < 4; leg++)
    {
        for(int axis = 0; axis < 3; axis++)
        {
            _f[leg][axis] = q_soln[leg*3 + axis];
            //std::cout << _f[leg][axis] << "  ";
        }
        //std::cout << std::endl;
    }
}

void ConvexMPCSolver::setMdesiredV(double mdesiredVx, double mdesiredVy) {
    ConvexMPCSolver::mdesiredVx = mdesiredVx;
    ConvexMPCSolver::mdesiredVy = mdesiredVy;
}
