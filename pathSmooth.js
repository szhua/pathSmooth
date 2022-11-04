import LatLng from '@/common/LatLng.js'
export default function PathSmooth() {


	var mIntensity = 3;
	var mThreshhold = 1.0; //0.3

	var mNoiseThreshhold = 20; //10

	/**
	 * 轨迹平滑优化
	 * @param originlist 原始轨迹list,list.size大于2
	 * @return 优化后轨迹list
	 */
	this.pathOptimize = function(originlist) {
		let list = removeNoisePoint(originlist); //去噪
		let afterList = kalmanFilterPath(list, mIntensity); //滤波
		let pathoptimizeList = reducerVerticalThreshold(afterList, mThreshhold); //抽稀
		return pathoptimizeList;
	}




	/**
	 * 轨迹去噪，删除垂距大于20m的点
	 * @param originlist 原始轨迹list,list.size大于2
	 * @return
	 */
	function removeNoisePoint(originlist) {
		return reduceNoisePoint(originlist, mNoiseThreshhold);
	}

	/********************************************************************************************************/
	/**
	 * 轨迹线路滤波
	 * @param originlist 原始轨迹list,list.size大于2
	 * @param intensity 滤波强度（1—5）
	 * @return
	 */
	function kalmanFilterPath(originlist, intensity = mIntensity) {
		let kalmanFilterList = [];
		if (originlist == null || originlist.length <= 2) {
			return kalmanFilterList;
		}
		initial(); //初始化滤波参数
		let latLng = null;
		let lastLoc = originlist[0];
		kalmanFilterList.push(lastLoc);

		for (var i = 1; i < originlist.length; i++) {
			let curLoc = originlist[i];
			latLng = kalmanFilterPoint(lastLoc, curLoc, intensity);
			if (latLng != null) {
				kalmanFilterList.push(latLng);
				lastLoc = latLng;
			}
		}
		return kalmanFilterList;
	}

	/**
	 * 单点滤波
	 * @param lastLoc 上次定位点坐标
	 * @param curLoc 本次定位点坐标
	 * @param intensity 滤波强度（1—5）
	 * @return 滤波后本次定位点坐标值
	 */
	function kalmanFilterPoint(lastLoc, curLoc, intensity = mIntensity) {
		if (pdelt_x == 0 || pdelt_y == 0) {
			initial();
		}
		let kalmanLatlng = null;
		if (lastLoc == null || curLoc == null) {
			return kalmanLatlng;
		}
		if (intensity < 1) {
			intensity = 1;
		} else if (intensity > 5) {
			intensity = 5;
		}
		for (var i = 0; i < intensity; i++) {
			kalmanLatlng = kalmanFilter(lastLoc.longitude, curLoc.longitude, lastLoc.latitude, curLoc.latitude);
			//进行属性的赋值；
			kalmanLatlng.suspend = curLoc.suspend
			kalmanLatlng.index = curLoc.index
			curLoc = kalmanLatlng;
		}


		return kalmanLatlng;
	}



	/***************************卡尔曼滤波开始********************************/
	let lastLocation_x; //上次位置
	let currentLocation_x; //这次位置
	let lastLocation_y; //上次位置
	let currentLocation_y; //这次位置
	let estimate_x; //修正后数据
	let estimate_y; //修正后数据
	let pdelt_x; //自预估偏差
	let pdelt_y; //自预估偏差
	let mdelt_x; //上次模型偏差
	let mdelt_y; //上次模型偏差
	let gauss_x; //高斯噪音偏差
	let gauss_y; //高斯噪音偏差
	let kalmanGain_x; //卡尔曼增益
	let kalmanGain_y; //卡尔曼增益

	let m_R = 0;
	let m_Q = 0;




	//初始模型
	function initial() {
		pdelt_x = 0.001;
		pdelt_y = 0.001;
		//        mdelt_x = 0;
		//        mdelt_y = 0;
		mdelt_x = 5.698402909980532E-4;
		mdelt_y = 5.698402909980532E-4;
	}

	function kalmanFilter(oldValue_x, value_x, oldValue_y, value_y) {
		lastLocation_x = oldValue_x;
		currentLocation_x = value_x;
		gauss_x = Math.sqrt(pdelt_x * pdelt_x + mdelt_x * mdelt_x) + m_Q; //计算高斯噪音偏差
		kalmanGain_x = Math.sqrt((gauss_x * gauss_x) / (gauss_x * gauss_x + pdelt_x * pdelt_x)) + m_R; //计算卡尔曼增益
		estimate_x = kalmanGain_x * (currentLocation_x - lastLocation_x) + lastLocation_x; //修正定位点
		mdelt_x = Math.sqrt((1 - kalmanGain_x) * gauss_x * gauss_x); //修正模型偏差

		lastLocation_y = oldValue_y;
		currentLocation_y = value_y;
		gauss_y = Math.sqrt(pdelt_y * pdelt_y + mdelt_y * mdelt_y) + m_Q; //计算高斯噪音偏差
		kalmanGain_y = Math.sqrt((gauss_y * gauss_y) / (gauss_y * gauss_y + pdelt_y * pdelt_y)) + m_R; //计算卡尔曼增益
		estimate_y = kalmanGain_y * (currentLocation_y - lastLocation_y) + lastLocation_y; //修正定位点
		mdelt_y = Math.sqrt((1 - kalmanGain_y) * gauss_y * gauss_y); //修正模型偏差

		let latlng = new LatLng(estimate_y, estimate_x);


		return latlng;
	}
	/***************************卡尔曼滤波结束**********************************/

	/***************************抽稀算法*************************************/
	function reducerVerticalThreshold(inPoints, threshHold = mThreshhold) {
		if (inPoints == null) {
			return null;
		}
		if (inPoints.length <= 2) {
			return inPoints;
		}
		var ret = [];
		for (var i = 0; i < inPoints.length; i++) {
			let pre = getLastLocation(ret);
			let cur = inPoints[i];
			if (pre == null || i == inPoints.length - 1) {
				ret.push(cur);
				continue;
			}
			let next = inPoints[i + 1];
			let distance = calculateDistanceFromPoint(cur, pre, next);
			if (distance > threshHold) {
				ret.push(cur);
			}
		}


		return ret;
	}

	function getLastLocation(oneGraspList) {
		if (oneGraspList == null || oneGraspList.length == 0) {
			return null;
		}
		let locListSize = oneGraspList.length;
		let lastLocation = oneGraspList[locListSize - 1];
		return lastLocation;
	}

	/**
	 * 计算当前点到线的垂线距离
	 * @param p 当前点
	 * @param lineBegin 线的起点
	 * @param lineEnd 线的终点
	 *
	 */
	function calculateDistanceFromPoint(p, lineBegin,
		lineEnd) {
		let A = p.longitude - lineBegin.longitude;
		let B = p.latitude - lineBegin.latitude;
		let C = lineEnd.longitude - lineBegin.longitude;
		let D = lineEnd.latitude - lineBegin.latitude;

		let dot = A * C + B * D;
		let len_sq = C * C + D * D;
		let param = dot / len_sq;

		var xx, yy;

		if (param < 0 || (lineBegin.longitude == lineEnd.longitude && lineBegin.latitude == lineEnd.latitude)) {
			xx = lineBegin.longitude;
			yy = lineBegin.latitude;
			//            return -1;
		} else if (param > 1) {
			xx = lineEnd.longitude;
			yy = lineEnd.latitude;
			//            return -1;
		} else {
			xx = lineBegin.longitude + param * C;
			yy = lineBegin.latitude + param * D;
		}
		return new PathSmooth().calculateLineDistance(p, new LatLng(yy, xx));
	}




	/***************************抽稀算法结束*********************************/

	function reduceNoisePoint(inPoints, threshHold = mNoiseThreshhold) {
		if (inPoints == null) {
			return null;
		}
		if (inPoints.length <= 2) {
			return inPoints;
		}
		var ret = [];

		for (var i = 0; i < inPoints.length; i++) {
			let pre = getLastLocation(ret);
			let cur = inPoints[i];
			if (pre == null || i == inPoints.length - 1) {
				ret.push(cur);
				continue;
			}
			let next = inPoints[i + 1];
			let distance = calculateDistanceFromPoint(cur, pre, next);
			if (distance < threshHold) {
				ret.push(cur);
			}
		}


		return ret;
	}


	/**
	 * inPoints包含了suspend标记；
	 * @param {Object} inPoints
	 */
	this.calulateTotalDistance = function(inPoints) {
		if (inPoints == null) {
			return 0.0;
		}
		if (inPoints.length <= 2) {
			return 0.0;
		}
		let totalDistance = 0.0


		var rets = []
		let currentRet = []
		inPoints.forEach((item, index) => {
			currentRet.push(item)
			//最后一个或者中断点的情况
			if (item.suspend === "0" || index === inPoints.length - 1) {
				//大于两个点的情况下；
				if (currentRet.length >= 2) {
					rets.push(currentRet)
				}
				currentRet = []
			}
		})

		rets.forEach(ret => {
			for (var i = 0; i < ret.length; i++) {
				let pre = null
				if (i > 0) {
					pre = ret[i - 1]
				}
				let cur = ret[i];
				if (pre == null) {
					continue;
				}
				let distance = new PathSmooth().calculateLineDistance(cur, pre);
				totalDistance += distance
			}
		})
		return totalDistance
	}




	this.splitSuspendData = function(inPoints) {
		var rets = []
		let currentRet = []
		inPoints.forEach((item, index) => {
			currentRet.push(item)
			//最后一个或者中断点的情况
			if (item.suspend === "0" || index === inPoints.length - 1) {
				//大于两个点的情况下；
				rets.push(currentRet)
				currentRet = []
			}
		})
		return rets
	}




	this.calculateLineDistance = function(var0, var1) {
		if (var0 != null && var1 != null) {
			try {
				let var2 = var0.longitude;
				let var4 = var0.latitude;
				let var6 = var1.longitude;
				let var8 = var1.latitude;
				var2 *= 0.01745329251994329;
				var4 *= 0.01745329251994329;
				var6 *= 0.01745329251994329;
				var8 *= 0.01745329251994329;
				var var10 = Math.sin(var2);
				var var12 = Math.sin(var4);
				var var14 = Math.cos(var2);
				var var16 = Math.cos(var4);
				var var18 = Math.sin(var6);
				var var20 = Math.sin(var8);
				var var22 = Math.cos(var6);
				var var24 = Math.cos(var8);
				var var28 = [];
				var var29 = [];
				var28.push(var16 * var14);
				var28.push(var16 * var10);
				var28.push(var12)
				var29.push(var24 * var22)
				var29.push(var24 * var18)
				var29.push(var20)
				return Math.asin(Math.sqrt((var28[0] - var29[0]) * (var28[0] - var29[0]) + (var28[1] -
							var29[1]) * (var28[1] - var29[1]) + (var28[2] - var29[2]) * (var28[2] - var29[2])) /
						2.0) *
					12742001.579854401;
			} catch (var26) {
				console.error(var26)
				return 0.0;
			}
		} else {
			try {
				console.error("非法坐标值")
			} catch (var27) {
				onsole.error(var27)
				return 0.0;
			}
			return 0.0;
		}

	}


}
