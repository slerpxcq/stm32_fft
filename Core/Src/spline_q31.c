/*
 * spline_q31.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Admin
 */

#include "spline_q31.h"

const int32_t XTableQ[256] =
{
		0<<FRAC_BITS, 1<<FRAC_BITS, 2<<FRAC_BITS, 3<<FRAC_BITS, 4<<FRAC_BITS, 5<<FRAC_BITS, 6<<FRAC_BITS, 7<<FRAC_BITS,
		8<<FRAC_BITS, 9<<FRAC_BITS, 10<<FRAC_BITS, 11<<FRAC_BITS, 12<<FRAC_BITS, 13<<FRAC_BITS, 14<<FRAC_BITS, 15<<FRAC_BITS,
		16<<FRAC_BITS, 17<<FRAC_BITS, 18<<FRAC_BITS, 19<<FRAC_BITS, 20<<FRAC_BITS, 21<<FRAC_BITS, 22<<FRAC_BITS, 23<<FRAC_BITS,
		24<<FRAC_BITS, 25<<FRAC_BITS, 26<<FRAC_BITS, 27<<FRAC_BITS, 28<<FRAC_BITS, 29<<FRAC_BITS, 30<<FRAC_BITS, 31<<FRAC_BITS,
		32<<FRAC_BITS, 33<<FRAC_BITS, 34<<FRAC_BITS, 35<<FRAC_BITS, 36<<FRAC_BITS, 37<<FRAC_BITS, 38<<FRAC_BITS, 39<<FRAC_BITS,
		40<<FRAC_BITS, 41<<FRAC_BITS, 42<<FRAC_BITS, 43<<FRAC_BITS, 44<<FRAC_BITS, 45<<FRAC_BITS, 46<<FRAC_BITS, 47<<FRAC_BITS,
		48<<FRAC_BITS, 49<<FRAC_BITS, 50<<FRAC_BITS, 51<<FRAC_BITS, 52<<FRAC_BITS, 53<<FRAC_BITS, 54<<FRAC_BITS, 55<<FRAC_BITS,
		56<<FRAC_BITS, 57<<FRAC_BITS, 58<<FRAC_BITS, 59<<FRAC_BITS, 60<<FRAC_BITS, 61<<FRAC_BITS, 62<<FRAC_BITS, 63<<FRAC_BITS,
		64<<FRAC_BITS, 65<<FRAC_BITS, 66<<FRAC_BITS, 67<<FRAC_BITS, 68<<FRAC_BITS, 69<<FRAC_BITS, 70<<FRAC_BITS, 71<<FRAC_BITS,
		72<<FRAC_BITS, 73<<FRAC_BITS, 74<<FRAC_BITS, 75<<FRAC_BITS, 76<<FRAC_BITS, 77<<FRAC_BITS, 78<<FRAC_BITS, 79<<FRAC_BITS,
		80<<FRAC_BITS, 81<<FRAC_BITS, 82<<FRAC_BITS, 83<<FRAC_BITS, 84<<FRAC_BITS, 85<<FRAC_BITS, 86<<FRAC_BITS, 87<<FRAC_BITS,
		88<<FRAC_BITS, 89<<FRAC_BITS, 90<<FRAC_BITS, 91<<FRAC_BITS, 92<<FRAC_BITS, 93<<FRAC_BITS, 94<<FRAC_BITS, 95<<FRAC_BITS,
		96<<FRAC_BITS, 97<<FRAC_BITS, 98<<FRAC_BITS, 99<<FRAC_BITS, 100<<FRAC_BITS, 101<<FRAC_BITS, 102<<FRAC_BITS, 103<<FRAC_BITS,
		104<<FRAC_BITS, 105<<FRAC_BITS, 106<<FRAC_BITS, 107<<FRAC_BITS, 108<<FRAC_BITS, 109<<FRAC_BITS, 110<<FRAC_BITS, 111<<FRAC_BITS,
		112<<FRAC_BITS, 113<<FRAC_BITS, 114<<FRAC_BITS, 115<<FRAC_BITS, 116<<FRAC_BITS, 117<<FRAC_BITS, 118<<FRAC_BITS, 119<<FRAC_BITS,
		120<<FRAC_BITS, 121<<FRAC_BITS, 122<<FRAC_BITS, 123<<FRAC_BITS, 124<<FRAC_BITS, 125<<FRAC_BITS, 126<<FRAC_BITS, 127<<FRAC_BITS,
		128<<FRAC_BITS, 129<<FRAC_BITS, 130<<FRAC_BITS, 131<<FRAC_BITS, 132<<FRAC_BITS, 133<<FRAC_BITS, 134<<FRAC_BITS, 135<<FRAC_BITS,
		136<<FRAC_BITS, 137<<FRAC_BITS, 138<<FRAC_BITS, 139<<FRAC_BITS, 140<<FRAC_BITS, 141<<FRAC_BITS, 142<<FRAC_BITS, 143<<FRAC_BITS,
		144<<FRAC_BITS, 145<<FRAC_BITS, 146<<FRAC_BITS, 147<<FRAC_BITS, 148<<FRAC_BITS, 149<<FRAC_BITS, 150<<FRAC_BITS, 151<<FRAC_BITS,
		152<<FRAC_BITS, 153<<FRAC_BITS, 154<<FRAC_BITS, 155<<FRAC_BITS, 156<<FRAC_BITS, 157<<FRAC_BITS, 158<<FRAC_BITS, 159<<FRAC_BITS,
		160<<FRAC_BITS, 161<<FRAC_BITS, 162<<FRAC_BITS, 163<<FRAC_BITS, 164<<FRAC_BITS, 165<<FRAC_BITS, 166<<FRAC_BITS, 167<<FRAC_BITS,
		168<<FRAC_BITS, 169<<FRAC_BITS, 170<<FRAC_BITS, 171<<FRAC_BITS, 172<<FRAC_BITS, 173<<FRAC_BITS, 174<<FRAC_BITS, 175<<FRAC_BITS,
		176<<FRAC_BITS, 177<<FRAC_BITS, 178<<FRAC_BITS, 179<<FRAC_BITS, 180<<FRAC_BITS, 181<<FRAC_BITS, 182<<FRAC_BITS, 183<<FRAC_BITS,
		184<<FRAC_BITS, 185<<FRAC_BITS, 186<<FRAC_BITS, 187<<FRAC_BITS, 188<<FRAC_BITS, 189<<FRAC_BITS, 190<<FRAC_BITS, 191<<FRAC_BITS,
		192<<FRAC_BITS, 193<<FRAC_BITS, 194<<FRAC_BITS, 195<<FRAC_BITS, 196<<FRAC_BITS, 197<<FRAC_BITS, 198<<FRAC_BITS, 199<<FRAC_BITS,
		200<<FRAC_BITS, 201<<FRAC_BITS, 202<<FRAC_BITS, 203<<FRAC_BITS, 204<<FRAC_BITS, 205<<FRAC_BITS, 206<<FRAC_BITS, 207<<FRAC_BITS,
		208<<FRAC_BITS, 209<<FRAC_BITS, 210<<FRAC_BITS, 211<<FRAC_BITS, 212<<FRAC_BITS, 213<<FRAC_BITS, 214<<FRAC_BITS, 215<<FRAC_BITS,
		216<<FRAC_BITS, 217<<FRAC_BITS, 218<<FRAC_BITS, 219<<FRAC_BITS, 220<<FRAC_BITS, 221<<FRAC_BITS, 222<<FRAC_BITS, 223<<FRAC_BITS,
		224<<FRAC_BITS, 225<<FRAC_BITS, 226<<FRAC_BITS, 227<<FRAC_BITS, 228<<FRAC_BITS, 229<<FRAC_BITS, 230<<FRAC_BITS, 231<<FRAC_BITS,
		232<<FRAC_BITS, 233<<FRAC_BITS, 234<<FRAC_BITS, 235<<FRAC_BITS, 236<<FRAC_BITS, 237<<FRAC_BITS, 238<<FRAC_BITS, 239<<FRAC_BITS,
		240<<FRAC_BITS, 241<<FRAC_BITS, 242<<FRAC_BITS, 243<<FRAC_BITS, 244<<FRAC_BITS, 245<<FRAC_BITS, 246<<FRAC_BITS, 247<<FRAC_BITS,
		248<<FRAC_BITS, 249<<FRAC_BITS, 250<<FRAC_BITS, 251<<FRAC_BITS, 252<<FRAC_BITS, 253<<FRAC_BITS, 254<<FRAC_BITS, 255<<FRAC_BITS
};

// Q32.31 format
void arm_spline_init_q31(
        arm_spline_instance_q31 * S,
        arm_spline_type type,
  const int32_t * x,
  const int32_t * y,
        uint32_t n,
				int32_t * coeffs,
				int32_t * tempBuffer)
{
    /*** COEFFICIENTS COMPUTATION ***/
    /* Type (boundary conditions):
        - Natural spline          ( S1''(x1) = 0 ; Sn''(xn) = 0 )
        - Parabolic runout spline ( S1''(x1) = S2''(x2) ; Sn-1''(xn-1) = Sn''(xn) ) */

    /* (n-1)-long buffers for b, c, and d coefficients */
	int32_t * b = coeffs;
	int32_t * c = coeffs+(n-1);
	int32_t * d = coeffs+(2*(n-1));

	int32_t * u = tempBuffer;   /* (n-1)-long scratch buffer for u elements */
	int32_t * z = tempBuffer+(n-1); /* n-long scratch buffer for z elements */

	int32_t hi, hm1; /* h(i) and h(i-1) */
	int32_t Bi; /* B(i), i-th element of matrix B=LZ */
	int32_t li; /* l(i), i-th element of matrix L    */
	int32_t cp1; /* Temporary value for c(i+1) */
	int32_t tmp1, tmp2;

    int32_t i; /* Loop counter */

    S->x = x;
    S->y = y;
    S->n_x = n;

    /* == Solve LZ=B to obtain z(i) and u(i) == */

    /* -- Row 1 -- */
    /* B(0) = 0, not computed */
    /* u(1,2) = a(1,2)/a(1,1) = a(1,2) */
    if(type == ARM_SPLINE_NATURAL)
        u[0] = 0;  /* a(1,2) = 0 */
    else if(type == ARM_SPLINE_PARABOLIC_RUNOUT)
        u[0] = q_from_int(-1); /* a(1,2) = -1 */

    z[0] = 0;  /* z(1) = B(1)/a(1,1) = 0 always */

    /* -- Rows 2 to N-1 (N=n+1) -- */
    hm1 = x[1] - x[0]; /* Initialize h(i-1) = h(1) = x(2)-x(1) */

    for (i=1; i<(int32_t)n-1; i++)
    {

        /* Compute B(i) */
        hi = x[i+1]-x[i];

        tmp1 = q_from_int(3);
        tmp1 = q_mult(tmp1, y[i+1]-y[i]);
        tmp1 = q_div(tmp1, hi);
        tmp2 = q_from_int(3);
        tmp2 = q_mult(tmp2, y[i]-y[i-1]);
        tmp2 = q_div(tmp2, hm1);
        Bi = tmp1 - tmp2;
        //Bi = 3*(y[i+1]-y[i])/hi - 3*(y[i]-y[i-1])/hm1;

        /* l(i) = a(i)-a(i,i-1)*u(i-1) = 2[h(i-1)+h(i)]-h(i-1)*u(i-1) */
        tmp1 = q_from_int(2);
        tmp1 = q_mult(tmp1, hi+hm1);
        tmp2 = q_mult(hm1, u[i-1]);
        li = tmp1 - tmp2;
        //li = 2*(hi+hm1) - hm1*u[i-1];

        /* u(i) = a(i,i+1)/l(i) = h(i)/l(i) */
        u[i] = q_div(hi, li);
        //u[i] = hi/li;

        /* z(i) = [B(i)-h(i-1)*z(i-1)]/l(i) */
        tmp1 = q_mult(hm1, z[i-1]);
        tmp1 = Bi - tmp1;
        z[i] = q_div(tmp1, li);
        //z[i] = (Bi-hm1*z[i-1])/li;

        /* Update h(i-1) for next iteration */
        hm1 = hi;
    }

    /* -- Row N -- */
    /* l(N) = a(N,N)-a(N,N-1)u(N-1) */
    /* z(N) = [-a(N,N-1)z(N-1)]/l(N) */
    if(type == ARM_SPLINE_NATURAL)
    {
        /* li = 1;     a(N,N) = 1; a(N,N-1) = 0 */
        z[n-1] = 0; /* a(N,N-1) = 0 */
    }
    else if(type == ARM_SPLINE_PARABOLIC_RUNOUT)
    {
        li = q_from_int(1)+u[n-2];      /* a(N,N) = 1; a(N,N-1) = -1 */
        z[n-1] = q_div(z[n-2], li); /* a(N,N-1) = -1 */
    }

    /* == Solve UX = Z to obtain c(i) and    */
    /*    compute b(i) and d(i) from c(i) == */

    cp1 = z[n-1]; /* Initialize c(i+1) = c(N) = z(N) */

    for (i=n-2; i>=0; i--)
    {
        /* c(i) = z(i)-u(i+1)c(i+1) */
    	tmp1 = q_mult(u[i], cp1);
    	c[i] = z[i]-tmp1;
        //c[i] = z[i]-u[i]*cp1;

        hi = x[i+1]-x[i];
        /* b(i) = [y(i+1)-y(i)]/h(i)-h(i)*[c(i+1)+2*c(i)]/3 */
        tmp1 = q_div(y[i+1]-y[i], hi);
        tmp2 = q_mult(hi, cp1+q_mult(q_from_int(2), c[i]));
        tmp2 = q_div(tmp2, q_from_int(3));
        b[i] = tmp1 - tmp2;
        //b[i] = (y[i+1]-y[i])/hi-hi*(cp1+2*c[i])/3;

        /* d(i) = [c(i+1)-c(i)]/[3*h(i)] */
        tmp1 = cp1-c[i];
        tmp2 = q_mult(q_from_int(3), hi);
        d[i] = q_div(tmp1, tmp2);
        //d[i] = (cp1-c[i])/(3*hi);

        /* Update c(i+1) for next iteration */
        cp1 = c[i];
    }

    /* == Finally, store the coefficients in the instance == */

    S->coeffs = coeffs;
}

void arm_spline_q31(
        arm_spline_instance_q31 * S,
  const int32_t * xq,
	      q31_t * pDst,
        uint32_t blockSize)
{
    const int32_t * x = S->x;
    const int32_t * y = S->y;
    int32_t n = S->n_x;

    /* Coefficients (a==y for i<=n-1) */
    int32_t * b = (S->coeffs);
    int32_t * c = (S->coeffs)+(n-1);
    int32_t * d = (S->coeffs)+(2*(n-1));

    const int32_t * pXq = xq;
    int32_t blkCnt = (int32_t)blockSize;
    int32_t blkCnt2;
    int32_t i;
    int32_t x_sc;


    /* Create output for x(i)<x<x(i+1) */
    for (i=0; i<n-1; i++)
    {
        while( *pXq <= x[i+1] && blkCnt > 0 )
        {
            x_sc = *pXq++;

            int32_t tmp = x_sc-x[i];
            int32_t tmp2 = q_mult(tmp, tmp);
            int32_t tmp3 = q_mult(tmp2, tmp);

            int32_t acc = y[i];
            acc += q_mult(b[i], tmp);
            acc += q_mult(c[i], tmp2);
            acc += q_mult(d[i], tmp3);
            //*pDst = int_from_q(acc);
            *pDst = acc;
            //*pDst = y[i]+b[i]*(x_sc-x[i])+c[i]*(x_sc-x[i])*(x_sc-x[i])+d[i]*(x_sc-x[i])*(x_sc-x[i])*(x_sc-x[i]);

            pDst++;
            blkCnt--;
        }
    }

    /* Create output for remaining samples (x>=x(n)) */

    blkCnt2 = blkCnt;

    while(blkCnt2 > 0)
    {
        x_sc = *pXq++;

        int32_t tmp = x_sc-x[i-1];
        int32_t tmp2 = q_mult(tmp, tmp);
        int32_t tmp3 = q_mult(tmp2, tmp);

        int32_t acc = y[i-1];
        acc += q_mult(b[i-1], tmp);
        acc += q_mult(c[i-1], tmp2);
        acc += q_mult(d[i-1], tmp3);
        //*pDst = int_from_q(acc);
        *pDst = acc;

        //*pDst = y[i-1]+b[i-1]*(x_sc-x[i-1])+c[i-1]*(x_sc-x[i-1])*(x_sc-x[i-1])+d[i-1]*(x_sc-x[i-1])*(x_sc-x[i-1])*(x_sc-x[i-1]);

        pDst++;
        blkCnt2--;
    }
}


