//
// Created by hs on 22. 8. 16.
//

#include "convexMPC/MPCSolver.hpp"

extern pSHM sharedMemory;

MPCSolver::MPCSolver(const uint8_t& horizon)
    : mDt(HIGH_CONTROL_dT)
    , mAlpha(1e-6)
    , mFmax(200)
    , mMu(0.6)
    , mHorizon(horizon)
    , bIsTrotFirstRun(true)
{
//    mWeightMat << 5, 5, 5, 5, 5, 5, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.f;
    mWeightMat << 20, 20, 20, 50, 50, 100, 0.01, 0.01, 1, 1, 1, 1, 0.0;
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

void MPCSolver::SetTrajectory(CubicTrajectoryGenerator Trajectory[3])
{
    for(int horizon = 0; horizon < mHorizon ; horizon++)
    {


        if (sharedMemory->gaitState != STAND)
        {
            xd[horizon*13+9] = sharedMemory->baseDesiredVelocity[0];
            xd[horizon*13+10] = sharedMemory->baseDesiredVelocity[1];

            xd[horizon*13+3] += sharedMemory->baseDesiredVelocity[0]*mDt*horizon;
            xd[horizon*13+4] += sharedMemory->baseDesiredVelocity[1]*mDt*horizon;
            xd[horizon*13+5] = 0.32;
        }
        else
        {
            xd[horizon*13+3] = 0;
            xd[horizon*13+4] = 0;
            xd[horizon*13+5] = Trajectory[2].getPositionTrajectory(sharedMemory->localTime + horizon*mDt);
        }
    }
    sharedMemory->baseDesiredPosition[0] = xd[3];
    sharedMemory->baseDesiredPosition[1] = xd[4];
    sharedMemory->baseDesiredPosition[2] = xd[5];

    sharedMemory->baseDesiredVelocity[2] = xd[11];
}

void MPCSolver::GetMetrices(const Vec13<double>&  x0, const double mFoot[4][3])
{
    getStateSpaceMatrix(x0, mFoot);
    transformC2QP();
    L.diagonal() = mWeightMat.replicate(mHorizon,1);

    H = 2*(Bqp.transpose()*L*Bqp + mAlpha*K);
    g = 2*Bqp.transpose()*L*(Aqp*x0 - xd);

    int k = 0;
    for(int i = 0; i < mHorizon; i++){
        for(int16_t j = 0; j < 4; j++){
            U_b(5*k + 0) = 5e10;
            U_b(5*k + 1) = 5e10;
            U_b(5*k + 2) = 5e10;
            U_b(5*k + 3) = 5e10;
            U_b(5*k + 4) = mFmax*sharedMemory->gaitTable[i*4+j];
            k++;
        }
    }
    float mu = 1.f/mMu;
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
        if(!(NearZero(lb_qpoases[i]) && NearZero(ub_qpoases[i])))
        {
            continue;
        }

        double* c_row = &A_qpoases[i*num_variables];
        for(int j=0; j<num_variables; j++)
        {
            if(NearOne(c_row[j]))
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

void MPCSolver::GetGRF(Vec3<double> GRF[4])
{
//    std::cout << "=====GRF=====" << std::endl;
    for(int leg = 0; leg < 4; leg++)
    {
        for(int axis = 0; axis < 3; axis++)
        {
            GRF[leg][axis] = q_soln[leg*3 + axis];
//            std::cout << GRF[leg][axis] << "\t";
        }
//        std::cout << std::endl;
    }
//    std::cout << std::endl;
}

void MPCSolver::getStateSpaceMatrix(const Vec13<double>& x0, const double mFoot[4][3])
{
    double yc = cos(x0[2]);
    double ys = sin(x0[2]);

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
            R_feet(row, col) = mFoot[row][col];
        }
    }
    Bc.setZero();

    mBodyInertia = R_yaw*mBodyInertia*R_yaw.transpose();
    mBodyInertiaInverse = mBodyInertia.inverse();

    for(int n=0; n<4; n++)
    {
        Bc.block(6,n*3,3,3) = mBodyInertiaInverse*GetSkew(R_feet.row(n));
        Bc.block(9,n*3,3,3) = Eigen::Matrix3d::Identity() / BODYMASS;
    }
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
    xd.setZero();
    K.setIdentity();
    Aqp.setZero();
    Bqp.setZero();
    L.setZero();
    H.setZero();
    g.setZero();
    U_b.setZero();
    fmat.setZero();
    mBodyInertia.setZero();
    for (int idx=0; idx<3; idx++)
    {
        mBodyInertia(idx,idx) = BODY_INERTIA[idx];
    }
}
