#include "timeStepper.h"

timeStepper::timeStepper(const vector<shared_ptr<elasticRod>>& m_limbs)
{
    limbs = m_limbs;
    kl = 10; // lower diagonals
    ku = 10; // upper diagonals

    freeDOF = 0;
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    ldb = freeDOF;
    NUMROWS = 2 * kl + ku + 1;
    totalForce = new double[freeDOF];
    jacobianLen = (2 * kl + ku + 1) * freeDOF;
    jacobian = new double [jacobianLen];
    dx = new double[freeDOF];
    DX = VectorXd::Zero(freeDOF);
    nrhs = 1;
    ipiv = new int[freeDOF];
    info = 0;
}

timeStepper::~timeStepper()
{
    ;
}

double* timeStepper::getForce()
{
    return totalForce;
}

double* timeStepper::getJacobian()
{
    return jacobian;
}

void timeStepper::addForce(int ind, double p, int limb_idx)
{
    shared_ptr<elasticRod> limb = limbs[limb_idx];

    offset = offsets[limb_idx];

    if (limb->getIfConstrained(ind) == 0) // free dof
    {
        mappedInd = limb->fullToUnconsMap[ind];
        totalForce[mappedInd + offset] += p; // subtracting elastic force
        Force[mappedInd + offset] += p;
    }
}

void timeStepper::addJacobian(int ind1, int ind2, double p, int limb_idx)
{
    shared_ptr<elasticRod> limb = limbs[limb_idx];
    mappedInd1 = limb->fullToUnconsMap[ind1];
    mappedInd2 = limb->fullToUnconsMap[ind2];
    offset = offsets[limb_idx];
    if (limb->getIfConstrained(ind1) == 0 && limb->getIfConstrained(ind2) == 0) // both are free
    {
        Jacobian(mappedInd2+offset, mappedInd1+offset) += p;
    }
}

void timeStepper::addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) {
    shared_ptr<elasticRod> limb1 = limbs[limb_idx1];
    shared_ptr<elasticRod> limb2 = limbs[limb_idx2];
    mappedInd1 = limb1->fullToUnconsMap[ind1];
    mappedInd2 = limb2->fullToUnconsMap[ind2];
    int offset1 = offsets[limb_idx1];
    int offset2 = offsets[limb_idx2];
    if (limb1->getIfConstrained(ind1) == 0 && limb2->getIfConstrained(ind2) == 0) {
        Jacobian(mappedInd2 + offset2, mappedInd1 + offset1) += p;
    }
}

void timeStepper::setZero()
{
    for (int i=0; i < freeDOF; i++)
        totalForce[i] = 0;
    for (int i=0; i < jacobianLen; i++)
        jacobian[i] = 0;
    Force = VectorXd::Zero(freeDOF);
    Jacobian = MatrixXd::Zero(freeDOF,freeDOF);
}

void timeStepper::update()
{
    freeDOF = 0;
    offsets.clear();
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }
    ldb = freeDOF;
    delete [] totalForce;
    delete [] jacobian;
    delete [] dx;
    delete [] ipiv;

    totalForce = new double[freeDOF];
    jacobianLen = (2 * kl + ku + 1) * freeDOF;
    jacobian = new double [jacobianLen];
    dx = new double[freeDOF];
    ipiv = new int[freeDOF];
    DX = VectorXd::Zero(freeDOF);
    Force = VectorXd::Zero(freeDOF);
    Jacobian = MatrixXd::Zero(freeDOF,freeDOF);
    setZero();
}

void timeStepper::pardisoSolver()
{
    int n = freeDOF;
    int ia[n+1];
    ia[0] = 1;

    int temp = 0;
    for (int i =0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (Jacobian(i,j) != 0)
            {
                temp = temp + 1;
            }
        }
        ia[i+1] = temp+1;
    }

    int ja[ia[n]];
    double a[ia[n]];
    temp = 0;

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (Jacobian(i,j) != 0)
            {
                ja[temp] = j + 1;
                a[temp] = Jacobian(i,j);
                temp = temp + 1;
            }
        }
    }
    MKL_INT mtype = 11;       /* Real unsymmetric matrix */
    // Descriptor of main sparse matrix properties
    double b[n], x[n], bs[n], res, res0;
    MKL_INT nrhs = 1;     /* Number of right hand sides. */
    /* Internal solver memory pointer pt, */
    /* 32-bit: int pt[64]; 64-bit: long int pt[64] */
    /* or void *pt[64] should be OK on both architectures */
    void *pt[64];
    /* Pardiso control parameters. */
    MKL_INT iparm[64];
    MKL_INT maxfct, mnum, phase, error, msglvl;
    /* Auxiliary variables. */
    MKL_INT i, j;
    double ddum;          /* Double dummy */
    MKL_INT idum;         /* Integer dummy. */
/* -------------------------------------------------------------------- */
/* .. Setup Pardiso control parameters. */
/* -------------------------------------------------------------------- */
    for ( i = 0; i < 64; i++ )
    {
        iparm[i] = 0;
    }
    iparm[0] = 0;         /* No solver default */
    iparm[1] = 2;         /* Fill-in reordering from METIS */
    iparm[3] = 0;         /* No iterative-direct algorithm */
    iparm[4] = 0;         /* No user fill-in reducing permutation */
    iparm[5] = 0;         /* Write solution into x */
    iparm[6] = 0;         /* Not in use */
    iparm[7] = 2;         /* Max numbers of iterative refinement steps */
    iparm[8] = 0;         /* Not in use */
    iparm[9] = 13;        /* Perturb the pivot elements with 1E-13 */
    iparm[10] = 1;        /* Use nonsymmetric permutation and scaling MPS */
    iparm[11] = 0;        /* Conjugate transposed/transpose solve */
    iparm[12] = 1;        /* Maximum weighted matching algorithm is switched-on (default for non-symmetric) */
    iparm[13] = 0;        /* Output: Number of perturbed pivots */
    iparm[14] = 0;        /* Not in use */
    iparm[15] = 0;        /* Not in use */
    iparm[16] = 0;        /* Not in use */
    iparm[17] = -1;       /* Output: Number of nonzeros in the factor LU */
    iparm[18] = -1;       /* Output: Mflops for LU factorization */
    iparm[19] = 0;        /* Output: Numbers of CG Iterations */


    maxfct = 1;           /* Maximum number of numerical factorizations. */
    mnum = 1;         /* Which factorization to use. */
    msglvl = 0;           /* Print statistical information  */
    error = 0;            /* Initialize error flag */
/* -------------------------------------------------------------------- */
/* .. Initialize the internal solver memory pointer. This is only */
/* necessary for the FIRST call of the PARDISO solver. */
/* -------------------------------------------------------------------- */
    for ( i = 0; i < 64; i++ )
    {
        pt[i] = 0;
    }
/* -------------------------------------------------------------------- */
/* .. Reordering and Symbolic Factorization. This step also allocates */
/* all memory that is necessary for the factorization. */
/* -------------------------------------------------------------------- */
    phase = 11;
    PARDISO (pt, &maxfct, &mnum, &mtype, &phase,
             &n, a, ia, ja, &idum, &nrhs, iparm, &msglvl, &ddum, &ddum, &error);
    if ( error != 0 )
    {
        printf ("\nERROR during symbolic factorization: " IFORMAT, error);
        exit (1);
    }
    // printf ("\nReordering completed ... ");
    // printf ("\nNumber of nonzeros in factors = " IFORMAT, iparm[17]);
    // printf ("\nNumber of factorization MFLOPS = " IFORMAT, iparm[18]);
/* -------------------------------------------------------------------- */
/* .. Numerical factorization. */
/* -------------------------------------------------------------------- */
    phase = 22;
    PARDISO (pt, &maxfct, &mnum, &mtype, &phase,
             &n, a, ia, ja, &idum, &nrhs, iparm, &msglvl, &ddum, &ddum, &error);
    if ( error != 0 )
    {
        printf ("\nERROR during numerical factorization: " IFORMAT, error);
        exit (2);
    }
    // printf ("\nFactorization completed ... ");
/* -------------------------------------------------------------------- */
/* .. Back substitution and iterative refinement. */
/* -------------------------------------------------------------------- */
    phase = 33;

// descrA.type = SPARSE_MATRIX_TYPE_GENERAL;
// descrA.mode = SPARSE_FILL_MODE_UPPER;
// descrA.diag = SPARSE_DIAG_NON_UNIT;
// mkl_sparse_d_create_csr ( &csrA, SPARSE_INDEX_BASE_ONE, n, n, ia, ia+1, ja, a );

    /* Set right hand side to one. */
    for ( i = 0; i < n; i++ )
    {
        b[i] = Force[i];
    }
//  Loop over 3 solving steps: Ax=b, AHx=b and ATx=b
    PARDISO (pt, &maxfct, &mnum, &mtype, &phase,
             &n, a, ia, ja, &idum, &nrhs, iparm, &msglvl, b, x, &error);
    if ( error != 0 )
    {
        printf ("\nERROR during solution: " IFORMAT, error);
        exit (3);
    }

    // printf ("\nThe solution of the system is: ");
    // for ( j = 0; j < n; j++ )
    // {
    //     printf ("\n x [" IFORMAT "] = % f", j, x[j]);
    // }
    // printf ("\n");

/* -------------------------------------------------------------------- */
/* .. Termination and release of memory. */
/* -------------------------------------------------------------------- */
    phase = -1;           /* Release internal memory. */
    PARDISO (pt, &maxfct, &mnum, &mtype, &phase,
             &n, &ddum, ia, ja, &idum, &nrhs,
             iparm, &msglvl, &ddum, &ddum, &error);

    for (int i = 0; i < n; i++)
    {
        dx[i] = x[i];
        DX[i] = x[i];
    }

    // auto stop = high_resolution_clock::now();
    // auto duration = duration_cast<microseconds>(stop - start);
    //
    // cout << "Time taken by function: "
    // 		 << duration.count() << " microseconds" << endl;

}
void timeStepper::integrator()
{
    pardisoSolver();
}
