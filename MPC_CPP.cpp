#include <iostream>
#include <cstdio>
#include <vector>
#include <random>
#include <fstream>

using namespace std;
/* Some static variables for the coefficients in the loss function */
static bool initialize_coefficients;
static double a1;
static double a2;
static double a3;
static double b1;
static double b2;
static double b3;

double loss_(vector<double> params, vector<vector<double>> current_position, vector<vector<double>> velocity_xy,
                    vector<double> target, vector<double> last_position,
                     vector<double> obstacle, unsigned int horizon) {
        double d1 = 0;
        double d2 = 0;
        double d3 = 0;
        
        for (unsigned int j = 0; j < horizon; j++) {
            
            velocity_xy[j][0] = params[j]; // first half elements in column 1
            
            velocity_xy[j][1] = params[horizon + j]; // next half elements in column 2
        }
        for (unsigned int i = 0; i < horizon; i++) {
            
            current_position[i][0] = last_position[0] + velocity_xy[i][0];
            current_position[i][1] = last_position[1] + velocity_xy[i][1];
            last_position[0] = current_position[i][0];
            last_position[1] = current_position[i][1];
            d1 += (pow((current_position[i][0] - obstacle[0]), 2) + pow((current_position[i][1] - obstacle[1]), 2));
            d2 += (pow((current_position[i][0] - target[0]), 2) + pow((current_position[i][1] - target[1]), 2));
        }
        
        if (!initialize_coefficients){ // the coefficients have to be tuned as per the problem.
            a1 = 1.6393e-3*d1;
            a2 = -1e-2*d2;
            a3 = -1.7643e-2*d2;
            b1 = -2.0765e-7*d1;
            b2 = -2.2684e-12*d2;
            b3 = -2.2684e-10*d2;
            initialize_coefficients = true;

        }
        d1 = a1*exp(b1*d1); 
        d2 = a2*exp(b2*d2);
        d3 = a3*exp(b3*d2);
        return (d1+d2+d3);
    }
class ModelPredictiveController{ // default constructor
    public:
        ModelPredictiveController(unsigned int horizon, vector<double> target_position,
                                    vector<double> obstacle_position, vector<double> initial_position) {   
            this->horizon = horizon;
            this->target = target_position;
            this->obstacle = obstacle_position;
            
            this->last_position = initial_position;
            
             } 
        virtual ~ModelPredictiveController() {} // destructor

    void initialize_matrix(vector<vector<double>> &vector_, unsigned int size_, vector<double>value = {0,0}) {
    
        for (unsigned int i = 0; i < size_; i++) {
            vector_.push_back(value);
        }
    }
    void initialize_vector(vector<double> &vector_, unsigned int size_) {
        for (unsigned int i = 0; i < size_; i++) {
            vector_.push_back(0);
        }
    }
    void reinitialize_params(vector<double> &vector_) {
        for (unsigned int i = 0; i < vector_.size(); i++) {
            vector_[i] = 0;
        }
    }
    void assign_vectors_and_matrices() {
        initialize_matrix(current_position, (unsigned int)(horizon));
        initialize_matrix(velocity_xy, (unsigned int)(horizon));
        initialize_vector(params, horizon*2);
    }
    
    double sum(vector<double> vector_) {
        double sum = 0;
        for (unsigned int i = 0; i < vector_.size(); i++) {
            sum+= vector_[i];
        }
        return sum;
    }
    vector<double> subtract (vector<double> a, vector<double> b) {
        vector<double> result = {0,0};
        for (unsigned int i = 0; i < a.size(); i++) {
            result[i] = abs(a[i] - b[i]);
        }
        return result;
    }

    vector<double> optimize_horizon(unsigned int iterations){
        random_device rd; // Generates random device.
        mt19937 gen(rd());
        uniform_real_distribution<> dis(-0.10, 0.10);
        
        vector<double> local_params;
        vector<double> local_current(2);
        double f1 = 0;
        double f2 = 0;
        
        
        local_current = last_position;
        for (unsigned int i = 0; i < iterations; i++) {
            
            f1 = loss_(params, current_position, velocity_xy, target, local_current, obstacle, horizon);
            local_params = params; // backup for updating
            for (unsigned int i = 0; i < params.size(); i++) {
                
		        local_params[i] = local_params[i] + dis(gen);
	        }
            
            f2 = loss_(local_params, current_position, velocity_xy, target, local_current, obstacle, horizon);
            
            for (unsigned int i = 0; i < params.size(); i++) {
                if (local_params[i] != 0){
                    if (((f2-f1)/(local_params[i] - params[i])) > 0) {
                        params[i] = params[i] -  1e-2; }
                    else if (((f2-f1)/(local_params[i] - params[i])) < 0) {
                        params[i] = params[i] +  1e-2;  
                    }
                } else { params[i] = 0;}           
            }

        }
        return params;
    }
    void write_to_file(vector<double> x, vector<double> y) {
        ofstream coordinates("C:\\Users\\Athrva Pandhare\\Desktop\\Connected Vehicles Matlab\\coordinates_mpc.csv");
        coordinates << "x,y\n";
        for (unsigned int i = 0; i < x.size(); i++) {
            cout<<"x : "<<x[i]<<"   "<<"y : "<<y[i]<<endl;
            coordinates<<x[i]<<','<<y[i]<<'\n';
        }
        coordinates.close();
    }
    void run_controller() {
        assign_vectors_and_matrices(); // getting initial values for all the vectors and matrices.
        vector<double> update_check = {0,0};
        vector<double> all_coordinates_x;
        vector<double> all_coordinates_y;
        unsigned int iterations = 55; // these are the inner iterations
        vector<double> criteria(2);
        double buffer_distance = 5.5;
	    criteria = subtract(target, last_position);

        while (sum(criteria) > 2.5) {
            
            reinitialize_params(params);

	        params = optimize_horizon(iterations);
            criteria = subtract(target, last_position);
            update_check[0] = last_position[0] + params[2];
            update_check[1] = last_position[1] + params[horizon + 2];
            if ((sum(criteria) + buffer_distance) > sum(subtract(target, update_check))) {
                for (unsigned int k = 0; k < (unsigned int)(params.size()/2); k++) {
                    if (k <= 2) {
                        last_position[0] +=  params[k];
                        last_position[1] +=  params[horizon + k];
                        all_coordinates_x.push_back(last_position[0]);
                        all_coordinates_y.push_back(last_position[1]);
                    }
                }           
            }    
            cout<< "Current Position : " << last_position[0] << ',' << last_position[1]<<endl;
        }
        write_to_file(all_coordinates_x, all_coordinates_y);
    }
    private:
    vector<vector<double>> velocity_xy; // velocity matrix (just a reshaping of the params vector.)
    vector<vector<double>> current_position; // current horizon positions
    unsigned int horizon;
    vector<double> target; // target coordinates
    vector<double> obstacle; // obstacle coordinates
    vector<double> params; // parameters for optimization
    vector<double> last_position;
    vector<double> min_global_container;
    double min_ = 1e6;
    };
int main(int argv, char** argc) {
    unsigned int horizon = 15;
    vector<double> target = {100,100};
    vector<double> obstacle = {20.0, 20.0};
    vector<double> initial = {0.0, 0.0};
    ModelPredictiveController* obj = new ModelPredictiveController(horizon, target, obstacle, initial);
    obj->run_controller();
    getchar();
}