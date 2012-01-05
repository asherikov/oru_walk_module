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
        const char *name)
{
    fprintf(file_op,"%s = [\n", name);
    for (unsigned int i=0; i < data_x.size(); i++)
    {
        fprintf(file_op, "%f %f;\n", data_x[i], data_y[i]);
    }
    fprintf(file_op, "];\n\n plot(%s(:,1), %s(:,2), 'k')\n", name, name);
}


void printVectors (
        FILE *file_op, 
        vector<double> &data_x, 
        vector<double> &data_y, 
        vector<double> &data_z, 
        const char *name)
{
    fprintf(file_op,"%s = [\n", name);
    for (unsigned int i=0; i < data_x.size(); i++)
    {
        fprintf(file_op, "%f %f %f;\n", data_x[i], data_y[i], data_z[i]);
    }
    fprintf(file_op, "];\n\n plot3(%s(:,1), %s(:,2), %s(:,3), 'r')\n", name, name, name);
}
