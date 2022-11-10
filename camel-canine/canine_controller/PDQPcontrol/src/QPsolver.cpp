//
// Created by hs on 22. 11. 9.
//

#include <PDQPcontrol/QPsolver.hpp>

extern pSHM sharedMemory;

PDQPsolver::PDQPsolver()
        : mAlpha(1e-6)
        , mForceMax(120)
        , mMu(0.6)
        , mBigNum(5e10)
        , mMass(14.2)
        , mKpLinear(100,100,100)
        , mKdLinear(10,10,10)
        , mKpAnguler(10,10,10)
        , mKdAngular(1,1,1)
{
    A.setZero();
    S.setZero();
    W.setZero();
    f.setZero();
    b.setZero();
    unit.setIdentity();
    gravity << 0,0,9.81;
    mBodyInertia << 0.11548, 0, 0, 0, 0.085609,0, 0,0,0.17193;

    mSweight << 1,1,1,0.1,0.1,0.1;
    mWweight << 1,1,1;

    U_b.setZero();
    L_b.setZero();
    fmat.setZero();

    H_qpoases = (qpOASES::real_t*)malloc(12*12*sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t*)malloc(12*sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t*)malloc(12*20*sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t*)malloc(20*sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t*)malloc(20*sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t*)malloc(12*sizeof(qpOASES::real_t));
}

PDQPsolver::~PDQPsolver()
{
    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
}

void PDQPsolver::SolveQP(const Vec13<double>& x0, const Vec13<double>& xd, const double mFoot[4][3])
{
    for(int idx=0; idx<3; idx++)
    {
        mDesiredAngularAccel[idx] = mKpAnguler[idx]*(xd[idx]-x0[idx]) + mKdAngular[idx]*(xd[idx+6]-x0[idx+6]);
        mDesiredLinearAccel[idx] = mKpLinear[idx]*(xd[idx+3]-x0[idx+3]) + mKdLinear[idx]*(xd[idx+9]-x0[idx+9]);
    }

    A.block(0,0,3,3) = unit;
    A.block(0,3,3,3) = unit;
    A.block(0,6,3,3) = unit;
    A.block(0,9,3,3) = unit;

    Eigen::Matrix<double,4,3> R_feet;
    for (int row=0; row<4; row++)
    {
        for (int col=0; col<3; col++)
        {
            R_feet(row, col) = mFoot[row][col];
        }
    }

    A.block(3,0,3,3) = GetSkew(R_feet.row(0));
    A.block(3,3,3,3) = GetSkew(R_feet.row(1));
    A.block(3,6,3,3) = GetSkew(R_feet.row(2));
    A.block(3,9,3,3) = GetSkew(R_feet.row(3));

    b.block(0,0,3,1) = mMass*(mDesiredLinearAccel+gravity);
    b.block(3,0,3,1) = mBodyInertia*mDesiredAngularAccel;

    S.diagonal() = mSweight;
    for(int idx=0; idx<4; idx++)
    {
        W(idx*3+0,idx*3+0) = mWweight[0];
        W(idx*3+1,idx*3+1) = mWweight[1];
        W(idx*3+2,idx*3+2) = mWweight[2];
    }

    H = 2*A.transpose()*S*A+mAlpha*W;
    g = -A.transpose()*S*b;

    setConstraints();
    solveQP();
}

void PDQPsolver::GetGRF(Vec3<double>* f)
{
    std::cout << "====GRF====" << std::endl;
    for(int leg = 0; leg < 4; leg++)
    {
        for(int axis = 0; axis < 3; axis++)
        {
            f[leg][axis] = q_soln[leg*3 + axis];
            std::cout << f[leg][axis] << "\t";
        }
        std::cout << std::endl;
    }
}


void PDQPsolver::setConstraints()
{
    for(int16_t idx = 0; idx < 4; idx++){
        U_b(5*idx + 0) = mBigNum;
        U_b(5*idx + 1) = mBigNum;
        U_b(5*idx + 2) = mBigNum;
        U_b(5*idx + 3) = mBigNum;
        U_b(5*idx + 4) = mForceMax*sharedMemory->gaitTable[idx];
    }

    float mu = 1.f/mMu;
    Eigen::Matrix<double,5,3> f_block;

    f_block <<  mu, 0,  1.f,
            -mu, 0,  1.f,
            0,  mu, 1.f,
            0, -mu, 1.f,
            0,   0, 1.f;

    for(int i = 0; i < 4; i++)
    {
        fmat.block(i*5,i*3,5,3) = f_block;
    }
}

void PDQPsolver::solveQP()
{
    transformMat2Real(H_qpoases, H, 12, 12);
    transformMat2Real(g_qpoases, g, 12, 1);
    transformMat2Real(A_qpoases, fmat, 20, 12);
    transformMat2Real(ub_qpoases,U_b, 20, 1);

    for(int i =0; i<20; i++)
    {
        lb_qpoases[i] = 0.f;
    }

    int16_t num_constraints = 20;
    int16_t num_variables = 12;

    qpOASES::int_t nWSR = 10000;
    qpOASES::QProblem problem(num_variables, num_constraints);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem.setOptions(op);
    problem.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, lb_qpoases, ub_qpoases, nWSR);

    int rval = problem.getPrimalSolution(q_soln);
    if(rval != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");

}

void PDQPsolver::transformMat2Real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols)
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