class Axis {
	private:
		float original_min, original_max, original_target;

		void set_values (float min_value, float max_value, float target_value);
		void find_scale();
		void find_offset();
		void set_scaled_values();

	public:
		float min, max, target;
		float scale, offset;
		
		Axis(float min_value,  float max_value, float target_value);
		float scale_value(float value);
};
