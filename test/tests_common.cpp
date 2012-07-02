/** 
 * @file
 * @author Alexander Sherikov
 * @date 05.01.2012 17:00:43 MSK
 */



/****************************************
 * INCLUDES 
 ****************************************/



/****************************************
 * FUNCTIONS 
 ****************************************/


void printVectors (
        FILE *file_op, 
        vector<double> &data_x, 
        vector<double> &data_y, 
        const char *name,
        const char *format)
{
    fprintf(file_op,"%s = [\n", name);
    for (unsigned int i=0; i < data_x.size(); i++)
    {
        fprintf(file_op, "%f %f;\n", data_x[i], data_y[i]);
    }
    fprintf(file_op, "];\n\n plot(%s(:,1), %s(:,2), '%s')\n", name, name, format);
}


void printVectors (
        FILE *file_op, 
        vector<double> &data_x, 
        vector<double> &data_y, 
        vector<double> &data_z, 
        const char *name,
        const char *format)
{
    fprintf(file_op,"%s = [\n", name);
    for (unsigned int i=0; i < data_x.size(); i++)
    {
        fprintf(file_op, "%f %f %f;\n", data_x[i], data_y[i], data_z[i]);
    }
    fprintf(file_op, "];\n\n plot3(%s(:,1), %s(:,2), %s(:,3), '%s')\n", name, name, name, format);
}




class test_log
{
    public:
        test_log(const char *filename)
        {
            file_op = fopen(filename, "a");
            fprintf(file_op,"hold on\n");
        }

        ~test_log()
        {
            fprintf(file_op,"hold off\n");
            fclose(file_op);
        }


        void addZMPrefPoint(const double x, const double y)
        {
            ZMPref_x.push_back(x);
            ZMPref_y.push_back(y);
        }
        void addZMPpoint(const double x, const double y)
        {
            ZMP_x.push_back(x);
            ZMP_y.push_back(y);
        }
        void addCoMpoint(const double x, const double y)
        {
            CoM_x.push_back(x);
            CoM_y.push_back(y);
        }
        void addFeetPositions (const nao_igm &nao)
        {
            left_foot_x.push_back(nao.left_foot_posture.data()[12]);
            left_foot_y.push_back(nao.left_foot_posture.data()[13]);
            left_foot_z.push_back(nao.left_foot_posture.data()[14]);
            right_foot_x.push_back(nao.right_foot_posture.data()[12]);
            right_foot_y.push_back(nao.right_foot_posture.data()[13]);
            right_foot_z.push_back(nao.right_foot_posture.data()[14]);
        }

        void flushLeftFoot ()
        {
            printVectors (file_op, left_foot_x, left_foot_y, left_foot_z, "LFP", "r");
        }
        void flushRightFoot ()
        {
            printVectors (file_op, right_foot_x, right_foot_y, right_foot_z, "RFP", "r");
        }
        void flushZMP ()
        {
            printVectors (file_op, ZMP_x, ZMP_y, "ZMP", "k");
        }
        void flushZMPref ()
        {
            printVectors (file_op, ZMPref_x, ZMPref_y, "ZMPref", "ko");
        }
        void flushCoM ()
        {
            printVectors (file_op, CoM_x, CoM_y, "CoM", "b");
        }



        FILE *file_op;

        vector<double> ZMP_x;
        vector<double> ZMP_y;
        vector<double> ZMPref_x;
        vector<double> ZMPref_y;
        vector<double> CoM_x;
        vector<double> CoM_y;

        vector<double> left_foot_x;
        vector<double> left_foot_y;
        vector<double> left_foot_z;
        vector<double> right_foot_x;
        vector<double> right_foot_y;
        vector<double> right_foot_z;
};
