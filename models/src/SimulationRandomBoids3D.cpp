#include "../include/SimulationRandomBoids3D.h"

namespace craal {

SimulationRandomBoids3D::SimulationRandomBoids3D() :
    g_sim(0), g_px(0), g_py(0), g_pz(0), g_gx(0), g_gy(0), g_gz(0), g_d(0),
    g_fileAz(""), g_fileAy(""), g_fileL(""), g_sampAz(500), g_sampAy(500), g_sampL(500)
{
    // 	addParam("Radius", 0.2, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
    // 	addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));

    addParam("Radius", 0.002, PDF(0.1, 1, PDF::NORMAL, 0.3, 0.2));
    addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));

    addConfigurable("AngleAroundZ", new Type_string, &g_fileAz);
    addConfigurable("AngleAroundY", new Type_string, &g_fileAy);
    addConfigurable("Length", new Type_string, &g_fileL);

    name = "RandomBoids3D";
}

SimulationRandomBoids3D::~SimulationRandomBoids3D()
{
    p_delete2();
}

void SimulationRandomBoids3D::p_delete2()
{
    if (g_sim) delete g_sim;
    if (g_px) delete [] g_px;
    if (g_py) delete [] g_py;
    if (g_pz) delete [] g_pz;
    if (g_gx) delete [] g_gx;
    if (g_gy) delete [] g_gy;
    if (g_gz) delete [] g_gz;
    if (g_d) delete [] g_d;
    g_sim = 0;
    g_px = 0;
    g_py = 0;
    g_pz = 0;
    g_gx = 0;
    g_gy = 0;
    g_gz = 0;
    g_d = 0;
}

void SimulationRandomBoids3D::updateSpecific()
{
    float tmp;
    float conv = 0.017453292519943295;

    if (changed("AngleAroundZ") && g_fileAz.compare("") != 0) {
        FILE * fd = fopen(g_fileAz.c_str(), "r");
        while (fscanf(fd, "%f", &tmp) == 1) g_sampAz.addValue(tmp*conv);
        fclose(fd);

        g_sampAz.init();

        std::cout<< g_fileAz<< std::endl;
        std::cout<< "["<< g_sampAz.getMin()<< "; "<< g_sampAz.getMax()<< "]"<< std::endl;
    }
    if (changed("AngleAroundY") && g_fileAy.compare("") != 0) {
        FILE * fd = fopen(g_fileAy.c_str(), "r");
        while (fscanf(fd, "%f", &tmp) == 1) g_sampAy.addValue(tmp*conv);
        fclose(fd);

        g_sampAy.init();

        std::cout<< g_fileAy<< std::endl;
        std::cout<< "["<< g_sampAy.getMin()<< "; "<< g_sampAy.getMax()<< "]"<< std::endl;
    }
    if (changed("Length") && g_fileL.compare("") != 0) {
        FILE * fd = fopen(g_fileL.c_str(), "r");
        while (fscanf(fd, "%f", &tmp) == 1) g_sampL.addValue(tmp*tmp/10000);
        fclose(fd);

        g_sampL.init();

        std::cout<< g_fileL<< std::endl;
        std::cout<< "["<< g_sampL.getMin()<< "; "<< g_sampL.getMax()<< "]"<< std::endl;
    }
}

void SimulationRandomBoids3D::init()
{
    p_delete2();

    g_sim = new helbing3d::HelbingSim();
    g_px = new float[g_nPedestrian];
    g_py = new float[g_nPedestrian];
    g_pz = new float[g_nPedestrian];
    g_gx = new float[g_nPedestrian];
    g_gy = new float[g_nPedestrian];
    g_gz = new float[g_nPedestrian];
    g_d = new float[g_nPedestrian];

    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->addAgent(0, 0, 0, 0, 0, 0);
        g_params[i*2+0] = 0.2;
        g_params[i*2+1] = 1.5;
        g_sim->setRadius(i, 0.2);

        g_d[i] = 0.0;
    }

    g_sim->doBoids = true;
    // 	g_sim->doBoids = false;
    g_sim->init();
}

void SimulationRandomBoids3D::reset()
{
    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->setRadius(i, g_params[i*2+0]);
    }
}

void SimulationRandomBoids3D::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    // 	g_sim->addObstacle(s_startx, s_starty, s_endx, s_endy);
}

void SimulationRandomBoids3D::setPosition(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    g_sim->setPos(s_indPedestrian, s_x, s_y, s_z);
}

void SimulationRandomBoids3D::setVelocity(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    if (s_x != 0.0 || s_y != 0.0 || s_z != 0.0) {
        g_px[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.x - s_x;
        g_py[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.y - s_y;
        g_pz[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.z - s_z;

        g_gx[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.x + s_x;
        g_gy[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.y + s_y;
        g_gz[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.z + s_z;
    }
    else {
        g_px[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.x - 1.0;
        g_py[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.y - 0.0;
        g_pz[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.z - 0.0;

        g_gx[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.x + 1.0;
        g_gy[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.y + 0.0;
        g_gz[s_indPedestrian] = g_sim->getAgent(s_indPedestrian).pos.z + 0.0;
    }

    g_sim->setVel(s_indPedestrian, s_x, s_y, s_z);
}

void SimulationRandomBoids3D::setGoal(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    // 	g_gx[s_indPedestrian] = s_x;
    // 	g_gy[s_indPedestrian] = s_y;
    // 	g_gz[s_indPedestrian] = s_z;
}

void SimulationRandomBoids3D::doStep(float s_dt)
{
    g_sim->setTimeStep(s_dt);

    float x, y, z, t, tmp;
    float dx, dy, dz, dtmp;
    float xx, xy, xz;
    float yx, yy, yz;
    float zx, zy, zz;
    float az, ay;
    float l;
    float a, b, c;

    bool cat = false;
    int agent = 8;
    float conv = 1/0.017453292519943295;

    float cx = 0;
    float cy = 0;
    float cz = 0;
    float radius = 2;
    radius = radius*radius;

    for (int i = 0; i < g_nPedestrian; i++) {
        cx += g_sim->getAgent(i).pos.x;
        cy += g_sim->getAgent(i).pos.y;
        cz += g_sim->getAgent(i).pos.z;
    }

    cx /= ((float)g_nPedestrian);
    cy /= ((float)g_nPedestrian);
    cz /= ((float)g_nPedestrian);

    for (int i = 0; i < g_nPedestrian; i++) {
        x = g_sim->getAgent(i).pos.x;
        y = g_sim->getAgent(i).pos.y;
        z = g_sim->getAgent(i).pos.z;

        xx = x - g_px[i];
        xy = y - g_py[i];
        xz = z - g_pz[i];

        dtmp = xx*xx + xy*xy + xz*xz;

        if (dtmp > g_d[i]) {
            if (
                    g_fileAz.compare("") != 0 &&
                    g_fileAy.compare("") != 0 &&
                    g_fileL.compare("") != 0
                    ) {
                az = g_sampAz.getSample();
                ay = g_sampAy.getSample();
                l = g_sampL.getSample();
            }
            else {
                az = 0;
                ay = 0;
                l = 10;
            }

            dx = cos(ay)*cos(az);
            dy = sin(az);
            dz = -sin(ay)*cos(az);

            if (i == agent) std::cout<< std::endl;
            if (i == agent) std::cout<< i<< " az = "<< az*conv<< "; ay = "<< ay*conv<< "; l = "<< l<< std::endl;
            if (i == agent && cat) std::cout<< i<< " dx = "<< dx<< "; dy = "<< dy<< "; dz = "<< dz<< std::endl;
            if (i == agent && cat) std::cout<< i<< " px = "<< g_px[i]<< "; py = "<< g_py[i]<< "; pz = "<< g_pz[i]<< std::endl;
            if (i == agent && cat) std::cout<< i<< " x = "<< x<< "; y = "<< y<< "; z = "<< z<< std::endl;

            dtmp = sqrt(dtmp);

            xx /= dtmp;
            xy /= dtmp;
            xz /= dtmp;

            if (i == agent && cat) std::cout<< i<< " xx = "<< xx<< "; xy = "<< xy<< "; xz = "<< xz<< std::endl;

            yx = -xy;
            yy = xx;
            yz = 0;

            dtmp = sqrt(yx*yx + yy*yy);

            yx /= dtmp;
            yy /= dtmp;
            yz /= dtmp;

            if (i == agent && cat) std::cout<< i<< " yx = "<< yx<< "; yy = "<< yy<< "; yz = "<< yz<< std::endl;

            zz = xy*yz - yy*xz;
            zy = xz*yx - yz*xx;
            zz = xx*yy - yx*xy;

            if (i == agent && cat) std::cout<< i<< " zx = "<< zx<< "; zy = "<< zy<< "; zz = "<< zz<< std::endl;

            a = dx*xx + dy*yx + dz*zx;
            b = dx*xy + dy*yy + dz*zy;
            c = dx*xz + dy*yz + dz*zz;

            a = 100*l*a + x;
            b = 100*l*b + y;
            c = 100*l*c + z;

            g_px[i] = x;
            g_py[i] = y;
            g_pz[i] = z;

            g_gx[i] = a;
            g_gy[i] = b;
            g_gz[i] = c;

            g_d[i] = l;

            if (i == agent && cat) std::cout<< i<< " a = "<< a<< "; b = "<< b<< ": c = "<< c<< std::endl;
        }

        dx = cx - x;
        dy = cy - y;
        dz = cz - z;

        dtmp = sqrt(dx*dx + dy*dy + dz*dz);

        if (dtmp > radius) {
            dx = -g_sim->getAgent(i).vel.x;
            dy = -g_sim->getAgent(i).vel.y;
            dz = -g_sim->getAgent(i).vel.z;

            g_sim->setVel(i, dx, dy, dz);

            g_sim->setPos(
                        i,
                        (x-cx)*0.99+cx,
                        (y-cy)*0.99+cy,
                        (z-cz)*0.99+cz
                        );

            g_px[i] = x;
            g_py[i] = y;
            g_pz[i] = z;

            g_gx[i] = x + 100.0*dx*g_d[i];
            g_gy[i] = y + 100.0*dy*g_d[i];
            g_gz[i] = z + 100.0*dz*g_d[i];
        }

        x = g_gx[i]-x;
        y = g_gy[i]-y;
        z = g_gz[i]-z;

        t = g_params[i*2+1];

        tmp = x*x+y*y+z*z;

        if (tmp > t*t) {
            t = sqrt(tmp);
            x /= t;
            y /= t;
            z /= t;
            x *= g_params[i*2+1];
            y *= g_params[i*2+1];
            z *= g_params[i*2+1];
        }
        g_sim->setGoalVel(i, x, y, z);
    }
    g_sim->doStep();
    for (int i = 0; i < g_nPedestrian; i++) {
        setNextState(
                    i,
                    g_sim->getAgent(i).pos.x,
                    g_sim->getAgent(i).pos.y,
                    g_sim->getAgent(i).pos.z,
                    g_sim->getAgent(i).vel.x,
                    g_sim->getAgent(i).vel.y,
                    g_sim->getAgent(i).vel.z
                    );
    }
}

}
