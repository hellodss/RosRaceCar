namespace Control
{
    class PID
    {
        public:
            double Kp, Ki, Kd;
            double error_out;
            double last_error;
            double integral;
            double inte_max;
            double last_diff;

            double PIDPositional(double error); //位置式PID
            double PIDIncremental(double error); //增量式PID
            void Init();
    }pid;

    double PID::PIDPositional(double error)
    {
        integral += error;
        if(integral > inte_max)
            integral = inte_max;

        error_out = Kp * error + Ki * integral + Kd * (error - last_error);
        last_error = error;
        return error_out;
    }

    double PID::PIDIncremental(double error)
    {
        error_out = Kp *(error - last_error) + Ki * error + Kd * ((error - last_error) - last_diff);
        last_diff = error - last_error;
        last_error = error;
        return error_out;
    }
    void PID::Init()
    {
        Kp = 15.0;
        Ki = 0.1;
        Kd = 3.0;
        error_out = 0.0;
        last_error = 0.0;
        integral = 0.0;
        inte_max = 8.0;
        last_diff = 0.0;
    }
    
}