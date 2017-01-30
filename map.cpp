
namespace map {
	// 6x6x6 meter space with 2 cm block
	unsigned char world[50 * 6][50 * 6][50 * 6]; // = {{{100}}};
	
	void init() {
		for (int i = 0; i < 300; i++) {
			for (int j = 0; j < 300; j++) {
				for (int k = 0; k < 300; k++) {
					world[i][j][k] = 255;
				}
			}
		}
	}

	void updateMap(cv::Mat& depthMap, cv::Point3i& position, cv::Mat& rotation) {
		// Foreach space between camera and raycast, increase value.

		// Foreach space behind raycast, decrease value.
	}
}