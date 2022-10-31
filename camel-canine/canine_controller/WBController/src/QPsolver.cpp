//
// Created by hs on 22. 10. 27.
//

#include <WBController/QPsolver.hpp>

extern pSHM sharedMemory;

QPsolver::QPsolver()
    : mDt(CONTROL_dT)
    , mAlpha(1e-10)
    , mBigNum(5e10)
    , mForceMax(240)
    , mMu(0.6)
{
    mWeightMat << 0.5, 0.5, 50, 20, 20, 80, 0, 0, 0.2, 0.05, 0.05, 0.05, 0.f;
    mYaw.setZero();
    mBodyInertia << 0.081321, 0, 0, 0, 0.060288,0, 0,0,0.12107;
    mBodyInertiaInverse.setZero();

    Ac.setZero();
    Bc.setZero();
    expmm.setZero();
    ABc.setZero();
    Aqp.setZero();
    Bqp.setZero();

    L.setZero();
    K.setZero();
    H.setZero();
    g.setZero();

    U_b.setZero();
    fmat.setZero();

    H_qpoases = (qpOASES::real_t*)malloc(12*12*sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t*)malloc(12*sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t*)malloc(12*20*sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t*)malloc(20*sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t*)malloc(20*sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t*)malloc(12*sizeof(qpOASES::real_t));

    H_red = (qpOASES::real_t*)malloc(12*12*sizeof(qpOASES::real_t));
    g_red = (qpOASES::real_t*)malloc(12*1*sizeof(qpOASES::real_t));
    A_red = (qpOASES::real_t*)malloc(12*20*sizeof(qpOASES::real_t));
    lb_red = (qpOASES::real_t*)malloc(20*1*sizeof(qpOASES::real_t));
    ub_red = (qpOASES::real_t*)malloc(20*1*sizeof(qpOASES::real_t));
    q_red = (qpOASES::real_t*)malloc(12*sizeof(qpOASES::real_t));
}

QPsolver::~QPsolver()
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

void QPsolver::SolveQP(const Vec13<double>& x0, const Vec13<double>& xd, const double mFoot[4][3])
{
    setStateSpaceMatrix(x0, mFoot);
    setWeights(x0, xd);
    setConstraints();
    solveQP();
}

void QPsolver::GetGRF(Vec3<double>* f)
{
    for(int leg = 0; leg < 4; leg++)
    {
        for(int axis = 0; axis < 3; axis++)
        {
            f[leg][axis] = q_soln[leg*3 + axis];
        }
    }
}

void QPsolver::setStateSpaceMatrix(const Vec13<double>& x0, const double mFoot[4][3])
{
    double yc = cos(x0[2]);
    double ys = sin(x0[2]);

    mYaw <<  yc,  -ys,   0,
            ys,  yc,   0,
            0,   0,   1;

    Ac.setZero();
    Ac(3,9) = 1.f;
    Ac(4,10) = 1.f;
    Ac(5,11) = 1.f;
    Ac(11,12) = 1.f;
    Ac.block(0,6,3,3) = mYaw.transpose();

    Eigen::Matrix<double,4,3> R_feet;
    for (int row=0; row<4; row++)
    {
        for (int col=0; col<3; col++)
        {
            R_feet(row, col) = mFoot[row][col] - x0[col+3];
        }
    }
    Bc.setZero();

    mBodyInertia = mYaw*mBodyInertia*mYaw.transpose();
    mBodyInertiaInverse = mBodyInertia.inverse();

    for(int n=0; n<4; n++)
    {
        Bc.block(6,n*3,3,3) = mBodyInertiaInverse*GetSkew(R_feet.row(n));
        Bc.block(9,n*3,3,3) = Eigen::Matrix3d::Identity() / 10.0;
    }
}

void QPsolver::setWeights(const Vec13<double>& x0, const Vec13<double>& xd)
{
    ABc.setZero();
    ABc.block(0,0,13,13) = Ac;
    ABc.block(0,13,13,12) = Bc;
    ABc = mDt*ABc;
    expmm = ABc.exp();

    Aqp = expmm.block(0,0,13,13);
    Bqp = expmm.block(0,13,13,12);

    L.diagonal() = mWeightMat;
    H = 2*(Bqp.transpose()*L*Bqp + mAlpha*K);
    g = 2*Bqp.transpose()*L*(Aqp*x0 - xd);
}

void QPsolver::setConstraints()
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

void QPsolver::solveQP()
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

void QPsolver::transformMat2Real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols)
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