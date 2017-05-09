#include "polishingtask.h"
#include <QString>
#include <QtXml>

using namespace Eigen;

const std::string CPolishingTask::VERSION_STR="V1.04";
/** \note v1.1: Added norminal vector information parser to the class.
 *
 */


CPolishingTask::CPolishingTask(double linePrecision ,double arcPrecision) {
    /**Setting polishing path discretization precision. line precision and arc precision should be separated.
     * original m_segmentPrecision just kept. But it won't be used.
     * the default value of linePrecision is -1,
     * the default value of arcPrecision is -1,
     * when the value less than zero, it means to automatically determine the two precision.
     * Now this determine algorithm is simple, and it can extend to more intelligent.
     */

    if(linePrecision>0) {
        m_segmentPrecision = linePrecision;
        m_LinePrecision = linePrecision;
    } else {
        m_segmentPrecision =0.001;
        m_LinePrecision =0.001;
    }
    if(arcPrecision>0) {
        m_ArcPrecision = arcPrecision;
    } else {
        m_ArcPrecision =0.0001;
    }
}

CPolishingTask::CPolishingTask(const std::string &ptdName, double linePrecision, double arcPrecision) {
    //save the ptd file path infomation.
    m_ptdPath = QString(ptdName.c_str());
    if(linePrecision>0) {
        m_segmentPrecision = linePrecision;
        m_LinePrecision = linePrecision;
    } else {
        m_segmentPrecision =0.001;
        m_LinePrecision =0.001;
    }
    if(arcPrecision>0) {
        m_ArcPrecision = arcPrecision;
    } else {
        m_ArcPrecision =0.0001;
    }
}

bool CPolishingTask::parsePTD(double linePrecision, double arcPrecision) {
    return parsePTD(m_ptdPath.toStdString(), linePrecision, arcPrecision);
}
bool CPolishingTask::parsePTD(const std::string &ptdName, double linePrecision, double arcPrecision) {
    /** segmentPrecision<0 represent automatic calculate optimal segment precision.
     *  the main function to parse PTD file.
     */

    QDomDocument doc;
    m_ptdPath = QString(ptdName.c_str());
    QFile file(m_ptdPath);
    /**
     * Setting precision to make prevent the value less than zero.
     */
    if(linePrecision>0) {
        m_segmentPrecision = linePrecision;
        m_LinePrecision = linePrecision;
    } else {
        m_segmentPrecision =0.001;
        m_LinePrecision =0.001;
    }
    if(arcPrecision>0) {
        m_ArcPrecision = arcPrecision;
    } else {
        m_ArcPrecision =0.0001;
    }

  if (!file.open(QIODevice::ReadOnly))
    return false;
  if (!doc.setContent(&file))
    return false;

    //get path list info from polishingtask/product/polishingpath.

    //Parse property
    QDomNodeList properties = doc.elementsByTagName("property");
    for (int i = 0; i < properties.size(); i++) {
        QString name = properties.item(i).toElement().attributeNode("name").nodeValue();
        QString value = properties.item(i).toElement().attributeNode("value").nodeValue();
        m_JSEngine.globalObject().setProperty(name, value.toDouble());
    }
    //Parse Workshop
    QDomElement workshop = doc.firstChildElement("polishingtask").firstChildElement("workshop");
    QDomNodeList polishers = workshop.elementsByTagName("polisher");
    for(int i=0;i< polishers.size();++i){
        QDomElement frame_node=polishers.item(i).firstChildElement("origin").toElement();
        m_polisher_frames.push_back(FrameParser(frame_node.attributeNode("xyz").nodeValue(),
                                      frame_node.attributeNode("rpy").nodeValue()));
    }

    //Parse product frame
    QDomElement robot = workshop.firstChildElement("robot");
    QDomElement origin = robot.firstChildElement("origin");
    m_RobotFrame = FrameParser(origin.attributeNode("xyz").nodeValue(), origin.attributeNode("rpy").nodeValue());

    //Parse product model path
    QDomElement product = doc.firstChildElement("polishingtask").firstChildElement("product");
    QString productModelSubPath= product.firstChildElement("geometry").firstChildElement("mesh")
            .attributeNode("filename").nodeValue();
    //TODO m_productModelPath=ros::package::getPath("cobot_description");
    //TODO ROS_INFO("cobot_polishing_task: The path of cobot_description package is: %s.",m_productModelPath.data());
    m_productModelPath.append(productModelSubPath.toStdString());

#if COBOT_DEBUG
    qDebug() << "m_productModelPath:" << QString::fromStdString(m_productModelPath);
#endif
    //m_origFrame = FrameParser(origin.attributeNode("xyz").nodeValue(), origin.attributeNode("rpy").nodeValue());
#if COBOT_DEBUG
    qDebug() << "m_origFrame: x = " << m_origFrame.p.x()
             << " y = " << m_origFrame.p.y()
             << " z = " << m_origFrame.p.z();
#endif
    QDomNodeList paths = product.firstChildElement("polishingpath").elementsByTagName("path");
#if COBOT_DEBUG
    qDebug() << "Paths size is:" << paths.size();
#endif
    static const QStringList tokenList = QStringList() << "LINE" << "ARC";

    for (int i = 0; i < paths.size(); i++) {
        Segment_Type segment;
        QDomNode n = paths.item(i);
        int polishing_id = n.firstChildElement("polishing_id").text().toInt();
        segment.curveType = (Curve_Type) tokenList.indexOf(n.firstChildElement("curve_type").text());
#if COBOT_DEBUG
        qDebug() << "curvetype:" << segment.curveType
                 << "text" << n.firstChildElement("curve_type").text();
#endif
        segment.startPoint = vectorParser(n.firstChildElement("startpoint").text());
#if COBOT_DEBUG
        qDebug() << "startPoint: x = " << segment.startPoint.x()
                 << " y = " << segment.startPoint.y()
                 << " z = " << segment.startPoint.z();
#endif
        segment.endPoint = vectorParser(n.firstChildElement("endpoint").text());
#if COBOT_DEBUG
        qDebug() << "endPoint: x = " << segment.endPoint.x()
                 << " y = " << segment.endPoint.y()
                 << " z = " << segment.endPoint.z();
#endif
        segment.startTangentDirection = vectorParser(n.firstChildElement("direction").text());
#if COBOT_DEBUG
        qDebug() << "startTangentDirection: x = " << segment.startTangentDirection.x()
                 << " y = " << segment.startTangentDirection.y()
                 << " z = " << segment.startTangentDirection.z();
#endif
        std::vector<double> fc = getNumList(n.firstChildElement("force").text());
        if (fc.size() == 2) {
            segment.startForce = fc[0];
            segment.endForce = fc[1];
        }
#if COBOT_DEBUG
        qDebug() << "startForce = " << segment.startForce
                 << "endForce = " << segment.endForce;
#endif
        push2LineSegmentList(segment);
    }
    updateNorminalDirction();
    updateTrajectory();
    return verifyPTD();
}

bool CPolishingTask::updateTrajectory(){
    /**
     * Update Trajecetory to ee frame.
     */
    foreach(PolishingSegment_Type polishingSegment,m_polishingSegments){
        Trajectory_Type traj;
        foreach(LineSegment_Type lineSegment, polishingSegment.data){
            //The trajectory here is underconstrained, I think it should determine at next part.
            KDL::Frame trajFrame=FrameParser(lineSegment.startPoint,lineSegment.nominalDirection);
            traj.push_back(m_ee_frame*trajFrame);
        }
        m_cartesianTrajectory.push_back(traj);
    }
    return true;
}

KDL::Frame CPolishingTask::FrameParser(Eigen::Vector3d point,Eigen::Vector3d axis){
    //Calculate rotation axis
    Eigen::Vector3d y_axis=Eigen::Vector3d(0,1,0);
    Eigen::Vector3d rot_axis;
    //Calculate rotation angle
    double angle;
    if (y_axis.dot(axis)>1-EPSILON) {
        rot_axis=Eigen::Vector3d(1,0,0);
        angle=0;
    } else if(y_axis.dot(axis)<-1+EPSILON){
        rot_axis=Eigen::Vector3d(1,0,0);
        angle=M_PI;
    } else {
        rot_axis=y_axis.cross(axis);
        rot_axis.normalize();
        angle=acos(y_axis.dot(axis));
    }
    KDL::Vector rotAxis(rot_axis.x(),rot_axis.y(),rot_axis.z());
    KDL::Vector vec(point.x(),point.y(),point.z());
    return KDL::Frame(KDL::Rotation::Rot(rotAxis,angle),vec);
}
bool CPolishingTask::updateNorminalDirction() {

    /**\brief access and load product model file.
     */

////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
////
////    pcl::PolygonMesh mesh;
////
////    if (pcl::io::loadPolygonFileSTL (m_productModelPath, mesh) == 0)
////    {
////        PCL_ERROR("Failed to load STL file\n");
////        return false;
////    }
////
////    /**\brief calculate norminal direction of each facet.
////     */
////    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
////    std::cout << "Loaded " << cloud->size () << " data points from fixed_shell.stl" << std::endl;
////    Eigen::Vector4f plane;
////    std::vector<Eigen::Vector4f> planes;
////    float curvature;
////    pcl::Vertices indices;
////    std::vector<int> indices_int(3);
////    foreach(indices,mesh.polygons){
////        copy(indices.vertices.begin(),indices.vertices.end(),indices_int.begin());
////        pcl::computePointNormal(*cloud,indices_int,plane,curvature);
////        planes.push_back(plane);
//#if COBOT_DEBUG
//            std::cout<<"plane: "<<endl<<plane<<endl<<"curvature: "<<curvature<<std::endl;
//#endif
//        }
//
//    Eigen::Vector3d middlePoint;
//
//    //Eigen::Hyperplane<double,3>::Through()
//
//    for(unsigned long k1=0;k1<m_polishingSegments.size();k1++){
//        PolishingSegment_Type& polishing_segment=m_polishingSegments.at(k1);
//        if(polishing_segment.curve_type==LINE){
//            for (int k2 = 0; k2 < polishing_segment.data.size(); ++k2)
//            {
//                middlePoint=(polishing_segment.data.at(k2).startPoint+polishing_segment.data.at(k2).endPoint)/2;
//                std::vector<double> distances;
//                double minDistance=EPSILON;
//                double distance=0;
//                long minIndex=-1;
//                for(unsigned long k=0;k<planes.size();k++){
//                    plane=planes.at(k);
//                    Eigen::Vector3d normVect(plane.coeff(0),plane.coeff(1),plane.coeff(2));
//                    distance=(middlePoint.dot(normVect)+plane.coeff(3))/normVect.norm();
//                    if(distance<0) distance=-distance;
//                    if(distance<minDistance){
//                        minDistance=distance;
//                        minIndex=k;
//                    }
//                }
//                if(minIndex>=0){
//                    plane=planes.at(minIndex);
//                    polishing_segment.data.at(k2).nominalDirection=Eigen::Vector3d(plane.coeff(0),plane.coeff(1),plane.coeff(2));
////                ROS_INFO("Segment ID: %d; point ID: %d; norminal direction: (%f,%f,%f)"
////                ,(int)k1,(int)k2,plane.coeff(0),plane.coeff(1),plane.coeff(2));
//                }
//            }
//        }else if(polishing_segment.curve_type==ARC){
//            //TODO add code for arc parser.
//            //parser at push2LineSegmentList().
//        }
//        else{
//            ROS_ERROR("Error: CPolishingTask::updateNorminalDirction()-Invalid curve type");
//        }
//    }

    /** Compute the Least-Squares plane fit for a given set of points,
     *  using their indices, and return the estimated plane parameters
     *  together with the surface curvature.
     *    template <typename PointT> inline void
     *    computePointNormal (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
     *    Eigen::Vector4f &plane_parameters, float &curvature)
     */
    // Save output
//    QString savepath =dir.absolutePath() + "/bun0-mls.pcd";
//    pcl::io::savePCDFileASCII(savepath.toStdString(), *cloud);
//    qDebug()<<path;
    return true;
}
bool CPolishingTask::push2LineSegmentList(Segment_Type segment) {
    /**
     * divide a whole segment to a list of short line segment.
     */
    LineSegment_Type lineSegment;
    PolishingSegment_Type polishing_segment;
    double forceStep;

    Eigen::Vector3d nextPoint = segment.startPoint;
    double nextForce = segment.startForce;
    //No auto-calculation at present.
    if(m_segmentPrecision<0) {
        m_segmentPrecision =0.001;
    }
    if(m_LinePrecision<0) {
        m_LinePrecision =0.001;
    }
    if(m_ArcPrecision<0) {
        m_ArcPrecision =0.0001;
    }
    /**
     * if segment curve type is LINE, then divided the segment uniformly.
     */
    if (segment.curveType == LINE) {
        Eigen::Vector3d pointStep = segment.endPoint - segment.startPoint;
        pointStep.normalize();
        pointStep *= m_LinePrecision;

        double length = (segment.endPoint - segment.startPoint).norm();
        forceStep = (segment.endForce - segment.startForce) / ceil(length / m_LinePrecision);

        for (int index = 0; index < length / m_LinePrecision; index++, nextForce += forceStep) {
            lineSegment.startPoint = nextPoint;
            lineSegment.endPoint = nextPoint+pointStep;
            nextPoint=lineSegment.endPoint;
            lineSegment.force = segment.startForce;
            polishing_segment.data.push_back(lineSegment);
        }

        //Note: Add the last point to segment to make different segment more continuous.
        lineSegment.startPoint=nextPoint;
        lineSegment.endPoint=segment.endPoint;
        lineSegment.force=segment.startForce;
        polishing_segment.data.push_back(lineSegment);

        polishing_segment.curve_type=LINE;
        m_polishingSegments.push_back(polishing_segment);

        return true;
    } else if (segment.curveType == ARC) {
        /**
         * if the segment curve type is ARC, then using length precision to calculate the angle step to get the next point
         * and norminal direction.
         */
        ArcFunction_Type arcSegment = arcParser(segment.startPoint, segment.endPoint,
                                                segment.startTangentDirection);
        double angleStep = m_ArcPrecision / arcSegment.radius;

        forceStep = (segment.endForce - segment.startForce) / ceil(arcSegment.angle / angleStep);
        //TODO: who is first input of cross operation should be define.
        //TODO: the algorithm to determine the norminal direction is not that reasonable, should be modified combined with surface norminal information.
        Eigen::Vector3d nextDirection=segment.startTangentDirection.cross(arcSegment.normalDirection);

        AngleAxisd rotMatrix = AngleAxisd(angleStep,
            arcSegment.normalDirection);//Rotation Matrix for next point;
        for (int index = 0; index < arcSegment.angle / angleStep; index++, nextForce += forceStep) {
            lineSegment.startPoint = nextPoint;
            nextPoint = arcSegment.centerPoint + rotMatrix * (nextPoint - arcSegment.centerPoint);
            lineSegment.endPoint = nextPoint;
            lineSegment.force = nextForce;
            lineSegment.nominalDirection=nextDirection;
            nextDirection=rotMatrix*nextDirection;
            polishing_segment.data.push_back(lineSegment);
        }

        //Note: Add the last point to segment to make different segment more continuous.
        lineSegment.startPoint=nextPoint;
        lineSegment.endPoint=segment.endPoint;
        lineSegment.force=segment.startForce;
        lineSegment.nominalDirection=nextDirection;
        polishing_segment.data.push_back(lineSegment);

        polishing_segment.arc=arcSegment;
        polishing_segment.curve_type=ARC;
        m_polishingSegments.push_back(polishing_segment);
        return true;
    } else
        return false;

}

bool CPolishingTask::verifyPTD() {
    /**
     * check if PTD can not execute right.(IK calulate error, unreasonable and so on...)
     */
    return true;
}

KDL::Frame CPolishingTask::FrameParser(const QString xyzString, const QString rpyString) {
    /**
     * parse the frame from a xyzString and rpyString from ptd file.
     */
    //qDebug()<<"xyzString: "<<xyzString<<"rpyString"<<rpyString;
    Eigen::Vector3d rpy = vectorParser(rpyString);
    Eigen::Vector3d vect = vectorParser(xyzString);
    KDL::Vector xyz(vect.x(), vect.y(), vect.z());

    return KDL::Frame(KDL::Rotation::RPY(rpy.x(), rpy.y(), rpy.z()), xyz);
}

Eigen::Vector3d CPolishingTask::vectorParser(const QString vectorString) {
    /**
     * parse the vector from a string,like "0,0,1".
     * This support javascript expression.
     */
    std::vector<double> vect;
    vect = getNumList(vectorString);
    if (vect.size() == 3) {
        return Eigen::Vector3d(vect.at(0), vect.at(1), vect.at(2));
    } else {
#if COBOT_DEBUG
        qDebug() << "Parse " << vectorString << "error in CPolishingTask::vectorParser";
#endif
        return Eigen::Vector3d();
    }
}

std::vector<double> CPolishingTask::getNumList(const QString vectorString) {

    //This funcion can add feature that eval the expression'value like:${pi/2}.
    std::vector<double> ls;
    QStringList strlist = vectorString.split(" ", QString::SkipEmptyParts);
#if COBOT_DEBUG
    qDebug() << vectorString;
#endif
        foreach(QString num, strlist) {
        if (num.left(2) == "${" && num.right(1) == "}") {
#if COBOT_DEBUG
            qDebug() << num;
#endif
            num = num.mid(2, num.length() - 3);
#if COBOT_DEBUG
            qDebug() << num;
#endif
            QJSValue exp_value = m_JSEngine.evaluate(num);

            if (exp_value.isNumber()) {
                ls.push_back(exp_value.toNumber());
            } else {
                return std::vector<double>();
            }

        } else {

            bool isOK;
            double result = num.toDouble(&isOK);

            if (isOK) {
                ls.push_back(result);
            } else {
                return std::vector<double>();
            }
        }
    }
    return ls;
}

bool CPolishingTask::write2XML(std::string path) {
    /**
     * write the polishingSegments result to a xml document.
     *
     * ref:
     *      http://www.java2s.com/Code/Cpp/Qt/Savecreatedxmldocumenttoafile.htm
     *      http://www.bogotobogo.com/Qt/Qt5_QtXML_QDomDocument_QDomElement.php
     * */

    QDomDocument document;
    QDomElement root = document.createElement("polishingsegments");
    document.appendChild(root);
    foreach(PolishingSegment_Type polishingSegment,m_polishingSegments){
        QDomElement nodePolishingSegemnt = document.createElement("polishingsegment");
            root.appendChild(nodePolishingSegemnt);
        foreach(LineSegment_Type seg, polishingSegment.data) {
            QDomElement NodeSegment = document.createElement("linesegment");
            NodeSegment.setAttribute("force", QString::number(seg.force));

            NodeSegment.setAttribute("endPoint", QString().sprintf("%f %f %f",
            seg.endPoint.x(), seg.endPoint.y(), seg.endPoint.z()));

            NodeSegment.setAttribute("startpoint", QString().sprintf("%f %f %f",
            seg.startPoint.x(), seg.startPoint.y(), seg.startPoint.z()));

            nodePolishingSegemnt.appendChild(NodeSegment);
        }
    }

    QFile outFile(QString::fromStdString(path));
    if (!outFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug("Failed to open file for writing.");
        return false;
    }

    QTextStream stream(&outFile);
    stream << document.toString();

    outFile.close();

    return true;
}


ArcFunction_Type CPolishingTask::arcParser(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint,
                                           Eigen::Vector3d startTangentDirection) {
    /**Calculate Rotation
     * Process:
     *   1.Calc 3 plane  get center of the circle.
     *   2.{spoint, norm},{spoint, sdirct},{epoint, edirct}
     *   3.[norm';sdirct';edirct']*cpoint=[norm'*spoint;sdirct'*spoint;edirct'*epoint]
     *   4.A*cpoint=b -> cpoint=A^(-1)*b;
     */

    //vector of start point to end point
    Eigen::Vector3d s2eVect = endPoint - startPoint;
    s2eVect.normalize();

    //tangent vector of start point
    Eigen::Vector3d sTangentDirection = startTangentDirection;
    sTangentDirection.normalize();

    //normal vector of circle plane
    Eigen::Vector3d normVect(sTangentDirection.cross(s2eVect));
    normVect.normalize();

    //the tangent vector of end point
    Eigen::Vector3d eTangentDirection =
            AngleAxisd(M_PI, s2eVect) * sTangentDirection;

    Eigen::Matrix3d A;
    A << normVect, sTangentDirection, eTangentDirection;
    A.transposeInPlace();

    Eigen::Vector3d b(normVect.dot(startPoint), sTangentDirection.dot(startPoint), eTangentDirection.dot(endPoint));

    Eigen::Vector3d centerPoint = A.inverse() * b;

    ArcFunction_Type result;
    result.normalDirection = normVect;
    result.centerPoint = centerPoint;
    result.startPoint = startPoint;
    result.radius = (startPoint - centerPoint).norm();
    result.angle =2 * acos(s2eVect.dot(sTangentDirection));
    //Direction need to test to fullfile the rotation direction.

    return result;
}