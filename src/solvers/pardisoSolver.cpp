#include "pardisoSolver.h"
#include "time_steppers/implicitTimeStepper.h"



pardisoSolver::pardisoSolver(shared_ptr<implicitTimeStepper> stepper) :
    baseSolver(stepper, solverType::PARDISO_SOLVER)
{
    mtype = 11;           /* Real unsymmetric matrix */

    maxfct = 1;           /* Maximum number of numerical factorizations. */
    mnum = 1;             /* Which factorization to use. */
    msglvl = 0;           /* Print statistical information  */
    error = 0;            /* Initialize error flag */

/* -------------------------------------------------------------------- */
/* .. Initialize the internal solver memory pointer. This is only */
/* necessary for the FIRST call of the PARDISO solver. */
/* -------------------------------------------------------------------- */
    for (int i = 0; i < 64; i++ )
    {
        pt[i] = nullptr;
    }

/* -------------------------------------------------------------------- */
/* .. Setup Pardiso control parameters. */
/* -------------------------------------------------------------------- */
    for (int i = 0; i < 64; i++ )
    {
        iparm[i] = 0;
    }
    iparm[0] = 1;         /* No solver default */
    iparm[1] = 2;         /* Fill-in reordering from METIS */
    iparm[3] = 0;         /* No iterative-direct algorithm */
    iparm[4] = 0;         /* No user fill-in reducing permutation */
    iparm[5] = 0;         /* Write solution into x */
    iparm[6] = 0;         /* Not in use */
    iparm[7] = 2;         /* Max numbers of iterative refinement steps */
    iparm[8] = 0;         /* Not in use */
    iparm[9] = 13;        /* Perturb the pivot elements with 1E-13 */
    iparm[10] = 0;        /* Use nonsymmetric permutation and scaling MPS */
    iparm[11] = 0;        /* Conjugate transposed/transpose solve */
    iparm[12] = 0;        /* Maximum weighted matching algorithm is switched-on (default for non-symmetric) */
    iparm[13] = 0;        /* Output: Number of perturbed pivots */
    iparm[14] = 0;        /* Not in use */
    iparm[15] = 0;        /* Not in use */
    iparm[16] = 0;        /* Not in use */
    iparm[17] = 0;        /* Output: Number of nonzeros in the factor LU */
    iparm[18] = 0;        /* Output: Mflops for LU factorization */
    iparm[19] = 0;        /* Output: Numbers of CG Iterations */
    iparm[23] = 10;       /* Two-level factorization for nonsymmetric matrices iparm[10 & 12] must be 0 */
}


pardisoSolver::~pardisoSolver() = default;


void pardisoSolver::integrator() {
    int n = stepper->freeDOF;

    MKL_INT* ia = stepper->ia;

    for (int i = 0; i < n-1; i++) {
        ia[i+2] += ia[i+1];
    }
    for (int i = 0; i < n; i++) {
        ia[i+1]++;
    }

    MKL_INT ja[ia[n]];
    double a[ia[n]];

    int ind = 0;
    sort(stepper->non_zero_elements.begin(), stepper->non_zero_elements.end());
    for (auto& id : stepper->non_zero_elements) {
        ja[ind] = id.second + 1;
        a[ind] = stepper->Jacobian(id.first, id.second);
        ind++;
    }

    /* Auxiliary variables. */
    double ddum;          /* Double dummy */
    MKL_INT idum;         /* Integer dummy. */

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

//  Loop over 3 solving steps: Ax=b, AHx=b and ATx=b
    PARDISO (pt, &maxfct, &mnum, &mtype, &phase,
             &n, a, ia, ja, &idum, &nrhs, iparm, &msglvl, stepper->force, stepper->dx, &error);
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
}
