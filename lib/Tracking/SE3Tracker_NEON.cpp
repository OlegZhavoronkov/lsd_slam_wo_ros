
#if defined(ENABLE_NEON)

namespace lsd_slam {

	float SE3Tracker::calcResidualAndBuffersNEON(
			const Eigen::Vector3f* refPoint,
			const Eigen::Vector2f* refColVar,
			int* idxBuf,
			int refNum,
			const std::shared_ptr<Frame> &frame,
			const Sophus::SE3f& referenceToFrame,
			int level,
			bool plotResidual)
	{
		return calcResidualAndBuffers(refPoint, refColVar, idxBuf, refNum, frame, referenceToFrame, level, plotResidual);
	}

	float SE3Tracker::calcWeightsAndResidualNEON(
			const Sophus::SE3f& referenceToFrame)
	{
		float tx = referenceToFrame.translation()[0];
		float ty = referenceToFrame.translation()[1];
		float tz = referenceToFrame.translation()[2];


		float constants[] = {
			tx, ty, tz, settings.var_weight,
			cameraPixelNoise2, settings.huber_d/2, -1, -1 // last values are currently unused
		};
		// This could also become a constant if one register could be made free for it somehow
		float cutoff_res_ponly4[4] = {10000, 10000, 10000, 10000}; // removed
		float* cur_buf_warped_z = buf_warped_z;
		float* cur_buf_warped_x = buf_warped_x;
		float* cur_buf_warped_y = buf_warped_y;
		float* cur_buf_warped_dx = buf_warped_dx;
		float* cur_buf_warped_dy = buf_warped_dy;
		float* cur_buf_warped_residual = buf_warped_residual;
		float* cur_buf_d = buf_d;
		float* cur_buf_idepthVar = buf_idepthVar;
		float* cur_buf_weight_p = buf_weight_p;
		int loop_count = buf_warped_size / 4;
		int remaining = buf_warped_size - 4 * loop_count;
		float sum_vector[] = {0, 0, 0, 0};

		float sumRes=0;


	#ifdef DEBUG
		loop_count = 0;
		remaining = buf_warped_size;
	#else
		if (loop_count > 0)
		{
			__asm__ __volatile__
			(
				// Extract constants
				"vldmia   %[constants], {q8-q9}              \n\t" // constants(q8-q9)
				"vdup.32  q13, d18[0]                        \n\t" // extract sigma_i2 x 4 to q13
				"vdup.32  q14, d18[1]                        \n\t" // extract huber_res_ponly x 4 to q14
				//"vdup.32  ???, d19[0]                        \n\t" // extract cutoff_res_ponly x 4 to ???
				"vdup.32  q9, d16[0]                         \n\t" // extract tx x 4 to q9, overwrite!
				"vdup.32  q10, d16[1]                        \n\t" // extract ty x 4 to q10
				"vdup.32  q11, d17[0]                        \n\t" // extract tz x 4 to q11
				"vdup.32  q8, d17[1]                         \n\t" // extract depthVarFac x 4 to q8, overwrite!

				"veor     q15, q15, q15                      \n\t" // set sumRes.sumResP(q15) to zero (by xor with itself)
				".loopcalcWeightsAndResidualNEON:            \n\t"

					"vldmia   %[buf_idepthVar]!, {q7}           \n\t" // s(q7)
					"vldmia   %[buf_warped_z]!, {q2}            \n\t" // pz(q2)
					"vldmia   %[buf_d]!, {q3}                   \n\t" // d(q3)
					"vldmia   %[buf_warped_x]!, {q0}            \n\t" // px(q0)
					"vldmia   %[buf_warped_y]!, {q1}            \n\t" // py(q1)
					"vldmia   %[buf_warped_residual]!, {q4}     \n\t" // rp(q4)
					"vldmia   %[buf_warped_dx]!, {q5}           \n\t" // gx(q5)
					"vldmia   %[buf_warped_dy]!, {q6}           \n\t" // gy(q6)

					"vmul.f32 q7, q7, q8                        \n\t" // s *= depthVarFac
					"vmul.f32 q12, q2, q2                       \n\t" // pz*pz (q12, temp)
					"vmul.f32 q3, q12, q3                       \n\t" // pz*pz*d (q3)

					"vrecpe.f32 q3, q3                          \n\t" // 1/(pz*pz*d) (q3)
					"vmul.f32 q12, q9, q2                       \n\t" // tx*pz (q12)
					"vmls.f32 q12, q11, q0                      \n\t" // tx*pz - tz*px (q12) [multiply and subtract] {free: q0}
					"vmul.f32 q0, q10, q2                       \n\t" // ty*pz (q0) {free: q2}
					"vmls.f32 q0, q11, q1                       \n\t" // ty*pz - tz*py (q0) {free: q1}
					"vmul.f32 q12, q12, q3                      \n\t" // g0 (q12)
					"vmul.f32 q0, q0, q3                        \n\t" // g1 (q0)

					"vmul.f32 q12, q12, q5                      \n\t" // gx * g0 (q12) {free: q5}
					"vldmia %[cutoff_res_ponly4], {q5}          \n\t" // cutoff_res_ponly (q5), load for later
					"vmla.f32 q12, q6, q0                       \n\t" // drpdd = gx * g0 + gy * g1 (q12) {free: q6, q0}

					"vmov.f32 q1, #1.0                          \n\t" // 1.0 (q1), will be used later

					"vmul.f32 q12, q12, q12                     \n\t" // drpdd*drpdd (q12)
					"vmul.f32 q12, q12, q7                      \n\t" // s*drpdd*drpdd (q12)
					"vadd.f32 q12, q12, q13                     \n\t" // sigma_i2 + s*drpdd*drpdd (q12)
					"vrecpe.f32 q12, q12                        \n\t" // w_p = 1/(sigma_i2 + s*drpdd*drpdd) (q12) {free: q7}

					// float weighted_rp = fabs(rp*sqrtf(w_p));
					"vrsqrte.f32 q7, q12                        \n\t" // 1 / sqrtf(w_p) (q7)
					"vrecpe.f32 q7, q7                          \n\t" // sqrtf(w_p) (q7)
					"vmul.f32 q7, q7, q4                        \n\t" // rp*sqrtf(w_p) (q7)
					"vabs.f32 q7, q7                            \n\t" // weighted_rp (q7)

					// float wh = fabs(weighted_rp < huber_res_ponly ? 1 : huber_res_ponly / weighted_rp);
					"vrecpe.f32 q6, q7                          \n\t" // 1 / weighted_rp (q6)
					"vmul.f32 q6, q6, q14                       \n\t" // huber_res_ponly / weighted_rp (q6)
					"vclt.f32 q0, q7, q14                       \n\t" // weighted_rp < huber_res_ponly ? all bits 1 : all bits 0 (q0)
					"vbsl     q0, q1, q6                        \n\t" // sets elements in q0 to 1(q1) where above condition is true, and to q6 where it is false {free: q6}
					"vabs.f32 q0, q0                            \n\t" // wh (q0)

					// sumRes.sumResP += wh * w_p * rp*rp
					"vmul.f32 q4, q4, q4                        \n\t" // rp*rp (q4)
					"vmul.f32 q4, q4, q12                       \n\t" // w_p*rp*rp (q4)
					"vmla.f32 q15, q4, q0                       \n\t" // sumRes.sumResP += wh*w_p*rp*rp (q15) {free: q4}

					// if(weighted_rp > cutoff_res_ponly)
					//     wh = 0;
					// *(buf_weight_p+i) = wh * w_p;
					"vcle.f32 q4, q7, q5                        \n\t" // mask in q4: ! (weighted_rp > cutoff_res_ponly)
					"vmul.f32 q0, q0, q12                       \n\t" // wh * w_p (q0)
					"vand     q0, q0, q4                        \n\t" // set q0 to 0 where condition for q4 failed (i.e. weighted_rp > cutoff_res_ponly)
					"vstmia   %[buf_weight_p]!, {q0}            \n\t"

				"subs     %[loop_count], %[loop_count], #1    \n\t"
				"bne      .loopcalcWeightsAndResidualNEON     \n\t"

				"vstmia   %[sum_vector], {q15}                \n\t"

			: /* outputs */ [buf_warped_z]"+&r"(cur_buf_warped_z),
							[buf_warped_x]"+&r"(cur_buf_warped_x),
							[buf_warped_y]"+&r"(cur_buf_warped_y),
							[buf_warped_dx]"+&r"(cur_buf_warped_dx),
							[buf_warped_dy]"+&r"(cur_buf_warped_dy),
							[buf_d]"+&r"(cur_buf_d),
							[buf_warped_residual]"+&r"(cur_buf_warped_residual),
							[buf_idepthVar]"+&r"(cur_buf_idepthVar),
							[buf_weight_p]"+&r"(cur_buf_weight_p),
							[loop_count]"+&r"(loop_count)
			: /* inputs  */ [constants]"r"(constants),
							[cutoff_res_ponly4]"r"(cutoff_res_ponly4),
							[sum_vector]"r"(sum_vector)
			: /* clobber */ "memory", "cc",
							"q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "q14", "q15"
			);

			sumRes += sum_vector[0] + sum_vector[1] + sum_vector[2] + sum_vector[3];
		}
	#endif

		for(int i=buf_warped_size-remaining; i<buf_warped_size; i++)
		{
			float px = *(buf_warped_x+i);	// x'
			float py = *(buf_warped_y+i);	// y'
			float pz = *(buf_warped_z+i);	// z'
			float d = *(buf_d+i);	// d
			float rp = *(buf_warped_residual+i); // r_p
			float gx = *(buf_warped_dx+i);	// \delta_x I
			float gy = *(buf_warped_dy+i);  // \delta_y I
			float s = settings.var_weight * *(buf_idepthVar+i);	// \sigma_d^2


			// calc dw/dd (first 2 components):
			float g0 = (tx * pz - tz * px) / (pz*pz*d);
			float g1 = (ty * pz - tz * py) / (pz*pz*d);


			// calc w_p
			float drpdd = gx * g0 + gy * g1;	// ommitting the minus
			float w_p = 1.0f / (cameraPixelNoise2 + s * drpdd * drpdd);
			float weighted_rp = fabs(rp*sqrtf(w_p));

			float wh = fabs(weighted_rp < (settings.huber_d/2) ? 1 : (settings.huber_d/2) / weighted_rp);

			sumRes += wh * w_p * rp*rp;

			*(buf_weight_p+i) = wh * w_p;
		}

		return sumRes / buf_warped_size;
	}

	void SE3Tracker::calculateWarpUpdateNEON(
			LGS6 &ls)
	{
	//	weightEstimator.reset();
	//	weightEstimator.estimateDistributionNEON(buf_warped_residual, buf_warped_size);
	//	weightEstimator.calcWeightsNEON(buf_warped_residual, buf_warped_weights, buf_warped_size);

		ls.initialize(width*height);

		float* cur_buf_warped_z = buf_warped_z;
		float* cur_buf_warped_x = buf_warped_x;
		float* cur_buf_warped_y = buf_warped_y;
		float* cur_buf_warped_dx = buf_warped_dx;
		float* cur_buf_warped_dy = buf_warped_dy;
		Vector6 v1,v2,v3,v4;
		float* v1_ptr;
		float* v2_ptr;
		float* v3_ptr;
		float* v4_ptr;
		for(int i=0;i<buf_warped_size;i+=4)
		{
			v1_ptr = &v1[0];
			v2_ptr = &v2[0];
			v3_ptr = &v3[0];
			v4_ptr = &v4[0];

			__asm__ __volatile__
			(
				"vldmia   %[buf_warped_z]!, {q10}            \n\t" // pz(q10)
				"vrecpe.f32 q10, q10                         \n\t" // z(q10)

				"vldmia   %[buf_warped_dx]!, {q11}           \n\t" // gx(q11)
				"vmul.f32 q0, q10, q11                       \n\t" // q0 = z*gx // = v[0]

				"vldmia   %[buf_warped_dy]!, {q12}           \n\t" // gy(q12)
				"vmul.f32 q1, q10, q12                       \n\t" // q1 = z*gy // = v[1]

				"vldmia   %[buf_warped_x]!, {q13}            \n\t" // px(q13)
				"vmul.f32 q5, q13, q12                       \n\t" // q5 = px * gy
				"vmul.f32 q5, q5, q10                        \n\t" // q5 = q5 * z = px * gy * z

				"vldmia   %[buf_warped_y]!, {q14}            \n\t" // py(q14)
				"vmul.f32 q3, q14, q11                       \n\t" // q3 = py * gx
				"vmls.f32 q5, q3, q10                        \n\t" // q5 = px * gy * z - py * gx * z // = v[5] (vmls: multiply and subtract from result)

				"vmul.f32 q10, q10, q10                      \n\t" // q10 = 1/(pz*pz)

				"vmul.f32 q6, q13, q11                       \n\t"
				"vmul.f32 q6, q6, q10                        \n\t" // q6 = val1 in SSE version = px * z_sqr * gx

				"vmul.f32 q7, q14, q12                       \n\t"
				"vmul.f32 q7, q7, q10                        \n\t" // q7 = val2 in SSE version = py * z_sqr * gy

				"vadd.f32 q2, q6, q7                         \n\t"
				"vneg.f32 q2, q2                             \n\t" // q2 = -px * z_sqr * gx -py * z_sqr * gy // = v[2]

				"vmul.f32 q8, q6, q14                        \n\t" // val3(q8) = px * z_sqr * gx * py
				"vadd.f32 q9, q12, q8                        \n\t" // val4(q9) = gy + px * z_sqr * gx * py
				"vmul.f32 q8, q7, q14                        \n\t" // val3(q8) = py * py * z_sqr * gy
				"vadd.f32 q9, q8, q9                         \n\t" // val4(q9) = gy + px * z_sqr * gx * py + py * py * z_sqr * gy
				"vneg.f32 q3, q9                             \n\t" // q3 = v[3]

				"vst4.32 {d0[0], d2[0], d4[0], d6[0]}, [%[v1]]! \n\t" // store v[0] .. v[3] for 1st value and inc pointer
				"vst4.32 {d0[1], d2[1], d4[1], d6[1]}, [%[v2]]! \n\t" // store v[0] .. v[3] for 2nd value and inc pointer
				"vst4.32 {d1[0], d3[0], d5[0], d7[0]}, [%[v3]]! \n\t" // store v[0] .. v[3] for 3rd value and inc pointer
				"vst4.32 {d1[1], d3[1], d5[1], d7[1]}, [%[v4]]! \n\t" // store v[0] .. v[3] for 4th value and inc pointer

				"vmul.f32 q8, q6, q13                        \n\t" // val3(q8) = px * px * z_sqr * gx
				"vadd.f32 q9, q11, q8                        \n\t" // val4(q9) = gx + px * px * z_sqr * gx
				"vmul.f32 q8, q7, q13                        \n\t" // val3(q8) = px * py * z_sqr * gy
				"vadd.f32 q4, q9, q8                         \n\t" // q4 = v[4]

				"vst2.32 {d8[0], d10[0]}, [%[v1]]               \n\t" // store v[4], v[5] for 1st value
				"vst2.32 {d8[1], d10[1]}, [%[v2]]               \n\t" // store v[4], v[5] for 2nd value
				"vst2.32 {d9[0], d11[0]}, [%[v3]]               \n\t" // store v[4], v[5] for 3rd value
				"vst2.32 {d9[1], d11[1]}, [%[v4]]               \n\t" // store v[4], v[5] for 4th value

	        : /* outputs */ [buf_warped_z]"+r"(cur_buf_warped_z),
			                [buf_warped_x]"+r"(cur_buf_warped_x),
			                [buf_warped_y]"+r"(cur_buf_warped_y),
			                [buf_warped_dx]"+r"(cur_buf_warped_dx),
			                [buf_warped_dy]"+r"(cur_buf_warped_dy),
			                [v1]"+r"(v1_ptr),
			                [v2]"+r"(v2_ptr),
			                [v3]"+r"(v3_ptr),
			                [v4]"+r"(v4_ptr)
	        : /* inputs  */
	        : /* clobber */ "memory", "cc", // TODO: is cc necessary?
		                    "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "q14"
			);


			// step 6: integrate into A and b:
			if(!(i+3>=buf_warped_size))
			{
				ls.update(v1, *(buf_warped_residual+i+0), *(buf_weight_p+i+0));
				ls.update(v2, *(buf_warped_residual+i+1), *(buf_weight_p+i+1));
				ls.update(v3, *(buf_warped_residual+i+2), *(buf_weight_p+i+2));
				ls.update(v4, *(buf_warped_residual+i+3), *(buf_weight_p+i+3));
			}
			else
			{
				ls.update(v1, *(buf_warped_residual+i+0), *(buf_weight_p+i+0));

				if(i+1>=buf_warped_size) break;
				ls.update(v2, *(buf_warped_residual+i+1), *(buf_weight_p+i+1));

				if(i+2>=buf_warped_size) break;
				ls.update(v3, *(buf_warped_residual+i+2), *(buf_weight_p+i+2));

				if(i+3>=buf_warped_size) break;
				ls.update(v4, *(buf_warped_residual+i+3), *(buf_weight_p+i+3));
			}
		}

		// solve ls
		ls.finish();
		//ls.solve(result);

	}

}


#endif
