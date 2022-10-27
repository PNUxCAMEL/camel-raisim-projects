//
// Created by hs on 22. 8. 16.
//

#include "convexMPC/MPCSolver.hpp"


extern pSHM sharedMemory;

MPCSolver::MPCSolver(const uint8_t& horizon)
    : mDt(CONTROL_dT)
    , mAlpha(ALPHA)
    , mWeightMat(WEIGHT)
    , mHorizon(horizon)
    , mBodyInertia(BODY_INERTIA)
{
    initMatrix();
    resizeMatrix();
}

MPCSolver::~MPCSolver() {
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

void MPCSolver::SetTrajectory(const double* mP)
{
    for(int i = 0; i < mHorizon ; i++)
    {
        xd(i*13+5,0) = 0.37;

        xd(i*13+3,0) = 0.0;
        xd(i*13+9,0) = 0.0;

        if(sharedMemory->gaitState == STAND)
        {
            xd(i*13+3,0) = mStopPosX;
            xd(i*13+9,0) = 0.0;
        }
        else
        {
            xd(i*13+3,0) = mP[0]+0.7*(mDt*i);
            xd(i*13+9,0) = 0.7;
        }
    }
}

void MPCSolver::GetMetrices(const double* mP, const double* mQ,
                            const double* mV, const double* mW,
                            const double mFoot[4][3])
{
    x0 << mQ[0],mQ[1],mQ[2],
          mP[0],mP[1],mP[2],
          mW[0],mW[1],mW[2],
          mV[0],mV[1],mV[2], GRAVITY;

    getStateSpaceMatrix(mP, mQ, mFoot);
    transformC2QP();
    L.diagonal() = mWeightMat.replicate(mHorizon,1);

    H = 2*(Bqp.transpose()*L*Bqp + mAlpha*K);
    g = 2*Bqp.transpose()*L*(Aqp*x0 - xd);

    int k = 0;
    for(int i = 0; i < mHorizon; i++){
        for(int16_t j = 0; j < 4; j++){
            U_b(5*k + 0) = BIG_NUMBER;
            U_b(5*k + 1) = BIG_NUMBER;
            U_b(5*k + 2) = BIG_NUMBER;
            U_b(5*k + 3) = BIG_NUMBER;
            U_b(5*k + 4) = F_MAX*sharedMemory->gaitTable[i*4+j];
            k++;
        }
    }
    float mu = 1.f/MU;
    Eigen::Matrix<double,5,3> f_block;

    f_block <<  mu, 0,  1.f,
            -mu, 0,  1.f,
            0,  mu, 1.f,
            0, -mu, 1.f,
            0,   0, 1.f;

    for(int i = 0; i < mHorizon*4; i++)
    {
        fmat.block(i*5,i*3,5,3) = f_block;
    }
}

int8_t near_zero(float a)
{
    return (a < 0.01 && a > -.01);
}

int8_t near_one(float a)
{
    return near_zero(a-1);
}

void MPCSolver::SolveQP()
{
    transformMat2Real(H_qpoases, H, 12*mHorizon, 12*mHorizon);
    transformMat2Real(g_qpoases, g, 12*mHorizon, 1);
    transformMat2Real(A_qpoases, fmat, 20*mHorizon, 12*mHorizon);
    transformMat2Real(ub_qpoases,U_b, 20*mHorizon, 1);

    for(int i =0; i<20*mHorizon; i++)
    {
        lb_qpoases[i] = 0.f;
    }

    int16_t num_constraints = 20*mHorizon;
    int16_t num_variables = 12*mHorizon;

    int new_cons = num_constraints;
    int new_vars = num_variables;

    for(int i=0; i<num_constraints; i++)
    {
        con_elim[i] = 0;
    }
    for(int i=0; i<num_variables; i++)
    {
        var_elim[i] = 0;
    }

    for(int i=0; i<num_constraints; i++)
    {
        if(!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i])))
        {
            continue;
        }

        double* c_row = &A_qpoases[i*num_variables];
        for(int j=0; j<num_variables; j++)
        {
            if(near_one(c_row[j]))
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

    for(int i=0; i<num_variables; i++)
    {
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

void MPCSolver::GetGRF(Vec3<double> _f[4]){
    for(int leg = 0; leg < 4; leg++)
    {
        for(int axis = 0; axis < 3; axis++)
        {
            _f[leg][axis] = q_soln[leg*3 + axis];
            std::cout << _f[leg][axis] << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void MPCSolver::GetJacobian(Eigen::Matrix<double,3,3>& J, double hip, double thigh, double calf, int side)
{
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

void MPCSolver::resizeMatrix()
{
    Aqp.resize(13*mHorizon, Eigen::NoChange);
    Bqp.resize(13*mHorizon, 12*mHorizon);
    L.resize(13*mHorizon, 13*mHorizon);
    xd.resize(13*mHorizon, Eigen::NoChange);
    U_b.resize(20*mHorizon, Eigen::NoChange);
    fmat.resize(20*mHorizon, 12*mHorizon);
    H.resize(12*mHorizon, 12*mHorizon);
    g.resize(12*mHorizon, Eigen::NoChange);
    K.resize(12*mHorizon, 12*mHorizon);

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

    H_qpoases = (qpOASES::real_t*)malloc(12*mHorizon*12*mHorizon*sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t*)malloc(12*mHorizon*sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t*)malloc(12*mHorizon*20*mHorizon*sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t*)malloc(20*mHorizon*sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t*)malloc(20*mHorizon*sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t*)malloc(12*mHorizon*sizeof(qpOASES::real_t));

    H_red = (qpOASES::real_t*)malloc(12*12*mHorizon*mHorizon*sizeof(qpOASES::real_t));
    g_red = (qpOASES::real_t*)malloc(12*1*mHorizon*sizeof(qpOASES::real_t));
    A_red = (qpOASES::real_t*)malloc(12*20*mHorizon*mHorizon*sizeof(qpOASES::real_t));
    lb_red = (qpOASES::real_t*)malloc(20*1*mHorizon*sizeof(qpOASES::real_t));
    ub_red = (qpOASES::real_t*)malloc(20*1*mHorizon*sizeof(qpOASES::real_t));
    q_red = (qpOASES::real_t*)malloc(12*mHorizon*sizeof(qpOASES::real_t));
    real_allocated = 1;
}

void MPCSolver::initMatrix()
{
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
}

void MPCSolver::transformC2QP()
{
    ABc.setZero();
    ABc.block(0,0,13,13) = Ac;
    ABc.block(0,13,13,12) = Bc;
    ABc = mDt*ABc;
    expmm = ABc.exp();

    Adt = expmm.block(0,0,13,13);
    Bdt = expmm.block(0,13,13,12);

    Eigen::Matrix<double,13,13> D[20];
    D[0].setIdentity();
    for(int i=1; i<=mHorizon; i++)
    {
        D[i] = Adt * D[i-1];
    }

    for(int r=0; r<mHorizon; r++)
    {
        Aqp.block(13*r,0,13,13) = D[r+1];
        for(int c=0; c<mHorizon; c++)
        {
            if(r>=c)
            {
                int a_num = r-c;
                Bqp.block(13*r,12*c,13,12) = D[a_num]*Bdt;
            }
        }
    }
}

inline Eigen::Matrix<double,3,3> getSkew(Vec3<double> r)
{
    Eigen::Matrix3d cm;
    cm << 0.f, -r(2), r(1),
            r(2), 0.f, -r(0),
            -r(1), r(0), 0.f;
    return cm;
}

void MPCSolver::getStateSpaceMatrix(const double* mP, const double* mQ, const double mFoot[4][3])
{
    double yc = cos(mQ[2]);
    double ys = sin(mQ[2]);

    R_yaw <<  yc,  -ys,   0,
            ys,  yc,   0,
            0,   0,   1;

    Ac.setZero();
    Ac(3,9) = 1.f;
    Ac(4,10) = 1.f;
    Ac(5,11) = 1.f;
    Ac(11,12) = 1.f;
    Ac.block(0,6,3,3) = R_yaw.transpose();

    Eigen::Matrix<double,4,3> R_feet;
    for (int row=0; row<4; row++)
    {
        for (int col=0; col<3; col++)
        {
            R_feet(row, col) = mFoot[row][col] - mP[col];
        }
    }
    Bc.setZero();

    mBodyInertia = R_yaw*mBodyInertia*R_yaw.transpose();
    mBodyInertiaInverse = mBodyInertia.inverse();

    for(int n=0; n<4; n++)
    {
        Bc.block(6,n*3,3,3) = mBodyInertiaInverse*getSkew(R_feet.row(n));
        Bc.block(9,n*3,3,3) = Eigen::Matrix3d::Identity() / BODYMASS;
    }
}

void MPCSolver::transformMat2Real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols)
{
    int32_t a = 0;
    for(int16_t r = 0; r < rows; r++)
    {
        for(int16_t c = 0; c < cols; c++)
        {
            dst[a] = src(r,c);
            a++;
        }
    }
}