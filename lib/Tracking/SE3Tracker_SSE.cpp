
#if defined(ENABLE_SSE)

namespace lsd_slam {

	float SE3Tracker::calcResidualAndBuffersSSE(
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


	float SE3Tracker::calcWeightsAndResidualSSE(
			const Sophus::SE3f& referenceToFrame)
	{
		const __m128 txs = _mm_set1_ps((float)(referenceToFrame.translation()[0]));
		const __m128 tys = _mm_set1_ps((float)(referenceToFrame.translation()[1]));
		const __m128 tzs = _mm_set1_ps((float)(referenceToFrame.translation()[2]));

		const __m128 zeros = _mm_set1_ps(0.0f);
		const __m128 ones = _mm_set1_ps(1.0f);


		const __m128 depthVarFacs = _mm_set1_ps((float)settings.var_weight);// float depthVarFac = var_weight;	// the depth var is over-confident. this is a constant multiplier to remedy that.... HACK
		const __m128 sigma_i2s = _mm_set1_ps((float)cameraPixelNoise2);


		const __m128 huber_res_ponlys = _mm_set1_ps((float)(settings.huber_d/2));

		__m128 sumResP = zeros;


		float sumRes = 0;

		for(int i=0;i<buf_warped_size-3;i+=4)
		{
	//		float px = *(buf_warped_x+i);	// x'
	//		float py = *(buf_warped_y+i);	// y'
	//		float pz = *(buf_warped_z+i);	// z'
	//		float d = *(buf_d+i);	// d
	//		float rp = *(buf_warped_residual+i); // r_p
	//		float gx = *(buf_warped_dx+i);	// \delta_x I
	//		float gy = *(buf_warped_dy+i);  // \delta_y I
	//		float s = depthVarFac * *(buf_idepthVar+i);	// \sigma_d^2


			// calc dw/dd (first 2 components):
			__m128 pzs = _mm_load_ps(buf_warped_z+i);	// z'
			__m128 pz2ds = _mm_rcp_ps(_mm_mul_ps(_mm_mul_ps(pzs, pzs), _mm_load_ps(buf_d+i)));  // 1 / (z' * z' * d)
			__m128 g0s = _mm_sub_ps(_mm_mul_ps(pzs, txs), _mm_mul_ps(_mm_load_ps(buf_warped_x+i), tzs));
			g0s = _mm_mul_ps(g0s,pz2ds); //float g0 = (tx * pz - tz * px) / (pz*pz*d);

			 //float g1 = (ty * pz - tz * py) / (pz*pz*d);
			__m128 g1s = _mm_sub_ps(_mm_mul_ps(pzs, tys), _mm_mul_ps(_mm_load_ps(buf_warped_y+i), tzs));
			g1s = _mm_mul_ps(g1s,pz2ds);

			 // float drpdd = gx * g0 + gy * g1;	// ommitting the minus
			__m128 drpdds = _mm_add_ps(
					_mm_mul_ps(g0s, _mm_load_ps(buf_warped_dx+i)),
					_mm_mul_ps(g1s, _mm_load_ps(buf_warped_dy+i)));

			 //float w_p = 1.0f / (sigma_i2 + s * drpdd * drpdd);
			__m128 w_ps = _mm_rcp_ps(_mm_add_ps(sigma_i2s,
					_mm_mul_ps(drpdds,
							_mm_mul_ps(drpdds,
									_mm_mul_ps(depthVarFacs,
											_mm_load_ps(buf_idepthVar+i))))));


			//float weighted_rp = fabs(rp*sqrtf(w_p));
			__m128 weighted_rps = _mm_mul_ps(_mm_load_ps(buf_warped_residual+i),
					_mm_sqrt_ps(w_ps));
			weighted_rps = _mm_max_ps(weighted_rps, _mm_sub_ps(zeros,weighted_rps));


			//float wh = fabs(weighted_rp < huber_res_ponly ? 1 : huber_res_ponly / weighted_rp);
			__m128 whs = _mm_cmplt_ps(weighted_rps, huber_res_ponlys);	// bitmask 0xFFFFFFFF for 1, 0x000000 for huber_res_ponly / weighted_rp
			whs = _mm_or_ps(
					_mm_and_ps(whs, ones),
					_mm_andnot_ps(whs, _mm_mul_ps(huber_res_ponlys, _mm_rcp_ps(weighted_rps))));




			// sumRes.sumResP += wh * w_p * rp*rp;
			if(i+3 < buf_warped_size)
				sumResP = _mm_add_ps(sumResP,
						_mm_mul_ps(whs, _mm_mul_ps(weighted_rps, weighted_rps)));

			// *(buf_weight_p+i) = wh * w_p;
			_mm_store_ps(buf_weight_p+i, _mm_mul_ps(whs, w_ps) );
		}
		sumRes = SSEE(sumResP,0) + SSEE(sumResP,1) + SSEE(sumResP,2) + SSEE(sumResP,3);

		return sumRes / ((buf_warped_size >> 2)<<2);
	}

	void SE3Tracker::calculateWarpUpdateSSE(
			LGS6 &ls)
	{
		ls.initialize( _imgSize.area() );

	//	printf("wupd SSE\n");
		for(int i=0;i<buf_warped_size-3;i+=4)
		{

			__m128 val1, val2, val3, val4;
			__m128 J61, J62, J63, J64, J65, J66;

			// redefine pz
			__m128 pz = _mm_load_ps(buf_warped_z+i);
			pz = _mm_rcp_ps(pz);						// pz := 1/z


			__m128 gx = _mm_load_ps(buf_warped_dx+i);
			val1 = _mm_mul_ps(pz, gx);			// gx / z => SET [0]
			//v[0] = z*gx;
			J61 = val1;



			__m128 gy = _mm_load_ps(buf_warped_dy+i);
			val1 = _mm_mul_ps(pz, gy);					// gy / z => SET [1]
			//v[1] = z*gy;
			J62 = val1;


			__m128 px = _mm_load_ps(buf_warped_x+i);
			val1 = _mm_mul_ps(px, gy);
			val1 = _mm_mul_ps(val1, pz);	//  px * gy * z
			__m128 py = _mm_load_ps(buf_warped_y+i);
			val2 = _mm_mul_ps(py, gx);
			val2 = _mm_mul_ps(val2, pz);	//  py * gx * z
			val1 = _mm_sub_ps(val1, val2);  // px * gy * z - py * gx * z => SET [5]
			//v[5] = -py * z * gx +  px * z * gy;
			J66 = val1;


			// redefine pz
			pz = _mm_mul_ps(pz,pz); 		// pz := 1/(z*z)

			// will use these for the following calculations a lot.
			val1 = _mm_mul_ps(px, gx);
			val1 = _mm_mul_ps(val1, pz);		// px * z_sqr * gx
			val2 = _mm_mul_ps(py, gy);
			val2 = _mm_mul_ps(val2, pz);		// py * z_sqr * gy


			val3 = _mm_add_ps(val1, val2);
			val3 = _mm_sub_ps(_mm_setr_ps(0,0,0,0),val3);	//-px * z_sqr * gx -py * z_sqr * gy
			//v[2] = -px * z_sqr * gx -py * z_sqr * gy;	=> SET [2]
			J63 = val3;


			val3 = _mm_mul_ps(val1, py); // px * z_sqr * gx * py
			val4 = _mm_add_ps(gy, val3); // gy + px * z_sqr * gx * py
			val3 = _mm_mul_ps(val2, py); // py * py * z_sqr * gy
			val4 = _mm_add_ps(val3, val4); // gy + px * z_sqr * gx * py + py * py * z_sqr * gy
			val4 = _mm_sub_ps(_mm_setr_ps(0,0,0,0),val4); //val4 = -val4.
			//v[3] = -px * py * z_sqr * gx +
			//       -py * py * z_sqr * gy +
			//       -gy;		=> SET [3]
			J64 = val4;


			val3 = _mm_mul_ps(val1, px); // px * px * z_sqr * gx
			val4 = _mm_add_ps(gx, val3); // gx + px * px * z_sqr * gx
			val3 = _mm_mul_ps(val2, px); // px * py * z_sqr * gy
			val4 = _mm_add_ps(val4, val3); // gx + px * px * z_sqr * gx + px * py * z_sqr * gy
			//v[4] = px * px * z_sqr * gx +
			//	   px * py * z_sqr * gy +
			//	   gx;				=> SET [4]
			J65 = val4;

			if(i+3<buf_warped_size)
			{
				ls.updateSSE(J61, J62, J63, J64, J65, J66, _mm_load_ps(buf_warped_residual+i), _mm_load_ps(buf_weight_p+i));
			}
			else
			{
				for(int k=0;i+k<buf_warped_size;k++)
				{
					Vector6 v6;
					v6 << SSEE(J61,k),SSEE(J62,k),SSEE(J63,k),SSEE(J64,k),SSEE(J65,k),SSEE(J66,k);
					ls.update(v6, *(buf_warped_residual+i+k), *(buf_weight_p+i+k));
				}
			}


		}

		// solve ls
		ls.finish();

	}


}

#endif
