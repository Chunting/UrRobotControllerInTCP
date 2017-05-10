#include "polishingtask.h"
#include <QString>
#include <QtXml>

using namespace Eigen;
using namespace KDL;
const std::string PolishingTask::VERSION_STR="V1.04";
/** \note v1.1: Added norminal vector information parser to the class.
 *
 */

PolishingTask::PolishingTask(double linePrecision, double arcPrecision) {
    //save the ptd file path infomation.

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
    m_model_data.clear();
}

bool PolishingTask::parsePTD(const std::string &ptdName, double linePrecision, double arcPrecision) {
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
    COBOT_LOG.debug() << "m_productModelPath:" << QString::fromStdString(m_productModelPath);

    //m_origFrame = FrameParser(origin.attributeNode("xyz").nodeValue(), origin.attributeNode("rpy").nodeValue());
//    COBOT_LOG.debug()  << "m_origFrame: x = " << m_origFrame.p.x()
//             << " y = " << m_origFrame.p.y()
//             << " z = " << m_origFrame.p.z();
    QDomNodeList paths = product.firstChildElement("polishingpath").elementsByTagName("path");
    COBOT_LOG.debug() << "Paths size is:" << paths.size();

    static const QStringList tokenList = QStringList() << "LINE" << "ARC";

    for (int i = 0; i < paths.size(); i++) {
        Segment_Type segment;
        QDomNode n = paths.item(i);
        int polishing_id = n.firstChildElement("polishing_id").text().toInt();
        segment.curveType = (Curve_Type) tokenList.indexOf(n.firstChildElement("curve_type").text());

        COBOT_LOG.debug() << "curvetype:" << segment.curveType
                 << "text" << n.firstChildElement("curve_type").text();

        segment.startPoint = vectorParser(n.firstChildElement("startpoint").text());

        COBOT_LOG.debug() << "startPoint: x = " << segment.startPoint.x()
                 << " y = " << segment.startPoint.y()
                 << " z = " << segment.startPoint.z();

        segment.endPoint = vectorParser(n.firstChildElement("endpoint").text());

        COBOT_LOG.debug() << "endPoint: x = " << segment.endPoint.x()
                 << " y = " << segment.endPoint.y()
                 << " z = " << segment.endPoint.z();

        segment.startTangentDirection = vectorParser(n.firstChildElement("direction").text());

        COBOT_LOG.debug() << "startTangentDirection: x = " << segment.startTangentDirection.x()
                 << " y = " << segment.startTangentDirection.y()
                 << " z = " << segment.startTangentDirection.z();

        std::vector<double> fc = getNumList(n.firstChildElement("force").text());
        if (fc.size() == 2) {
            segment.startForce = fc[0];
            segment.endForce = fc[1];
        }

        COBOT_LOG.debug() << "startForce = " << segment.startForce
                 << "endForce = " << segment.endForce;

        push2LineSegmentList(segment);
    }
    updateNorminalDirction();
    updateTrajectory();
    return verifyPTD();
}

bool PolishingTask::updateTrajectory(){
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

KDL::Frame PolishingTask::FrameParser(Eigen::Vector3d point,Eigen::Vector3d axis){
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
bool PolishingTask::updateNorminalDirction() {
    Eigen::Vector3d middlePoint;
    for(unsigned long k1=0;k1<m_polishingSegments.size();k1++){
        PolishingSegment_Type& polishing_segment=m_polishingSegments.at(k1);
        if(polishing_segment.curve_type==LINE){
            for (int k2 = 0; k2 < polishing_segment.data.size(); ++k2)
            {
                middlePoint=(polishing_segment.data.at(k2).startPoint+polishing_segment.data.at(k2).endPoint)/2;
                std::vector<double> distances;
                double minDistance=EPSILON;
                double distance=0;
                long minIndex=-1;
                Eigen::Vector4f plane;
                for(unsigned long k=0;k<m_model_data.size();k++){
                    plane=m_model_data.at(k).plane;
                    Eigen::Vector3d normVect(plane(0),plane(1),plane(2));
                    distance=(middlePoint.dot(normVect)+plane(3))/normVect.norm();
                    if(distance<0) distance=-distance;
                    if(distance<minDistance){
                        minDistance=distance;
                        minIndex=k;
                    }
                }
                if(minIndex>=0){
                    plane=m_model_data.at(minIndex).plane;
                    polishing_segment.data.at(k2).nominalDirection=Eigen::Vector3d(plane(0),plane(1),plane(2));
//                COBOT_LOG.notice()<<"Segment ID: %d; point ID: %d; norminal direction: (%f,%f,%f)"
//                ,(int)k1,(int)k2,plane(0),plane(1),plane(2);
                }
            }
        }else if(polishing_segment.curve_type==ARC){
            //TODO add code for arc parser.
            //parser at push2LineSegmentList().
        }
        else{
            COBOT_LOG.error()<<"Error: CPolishingTask::updateNorminalDirction()-Invalid curve type";
            return false;
        }
    }
    return true;
}
bool PolishingTask::push2LineSegmentList(Segment_Type segment) {
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

bool PolishingTask::verifyPTD() {
    /**
     * check if PTD can not execute right.(IK calulate error, unreasonable and so on...)
     */
    return true;
}

KDL::Frame PolishingTask::FrameParser(const QString xyzString, const QString rpyString) {
    /**
     * parse the frame from a xyzString and rpyString from ptd file.
     */
    //COBOT_LOG.debug()<<"xyzString: "<<xyzString<<"rpyString"<<rpyString;
    Eigen::Vector3d rpy = vectorParser(rpyString);
    Eigen::Vector3d vect = vectorParser(xyzString);
    KDL::Vector xyz(vect.x(), vect.y(), vect.z());

    return KDL::Frame(KDL::Rotation::RPY(rpy.x(), rpy.y(), rpy.z()), xyz);
}

Eigen::Vector3d PolishingTask::vectorParser(const QString vectorString) {
    /**
     * parse the vector from a string,like "0,0,1".
     * This support javascript expression.
     */
    std::vector<double> vect;
    vect = getNumList(vectorString);
    if (vect.size() == 3) {
        return Eigen::Vector3d(vect.at(0), vect.at(1), vect.at(2));
    } else {
        COBOT_LOG.debug() << "Parse " << vectorString << "error in CPolishingTask::vectorParser";
        return Eigen::Vector3d();
    }
}

std::vector<double> PolishingTask::getNumList(const QString vectorString) {

    //This funcion can add feature that eval the expression'value like:${pi/2}.
    std::vector<double> ls;
    QStringList strlist = vectorString.split(" ", QString::SkipEmptyParts);
    COBOT_LOG.debug() << vectorString;
        foreach(QString num, strlist) {
        if (num.left(2) == "${" && num.right(1) == "}") {
            COBOT_LOG.debug() << num;
            num = num.mid(2, num.length() - 3);
            COBOT_LOG.debug() << num;
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

bool PolishingTask::write2XML(std::string path) {
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
        COBOT_LOG.debug()<<"Failed to open file for writing.";
        return false;
    }

    QTextStream stream(&outFile);
    stream << document.toString();

    outFile.close();

    return true;
}


ArcFunction_Type PolishingTask::arcParser(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint,
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
void PolishingTask::parseSTL(std::string model_path){
    m_model_data.clear();
    /*** \ref https://en.wikipedia.org/wiki/STL_(file_format)
     *  UINT8[80] – Header
     *  UINT32 – Number of triangles
     *  foreach triangle
     *  REAL32[3] – Normal vector
     *  REAL32[3] – Vertex 1
     *  REAL32[3] – Vertex 2
     *  REAL32[3] – Vertex 3
     *  UINT16 – Attribute byte count
     *  end
     */

    QFile file(model_path.c_str());
    file.open(QIODevice::ReadOnly);
    file.seek(0);
    QDataStream in(&file);    // read the data serialized from the file
    if (in.status() != QDataStream::Ok ){
        COBOT_LOG.error()<<(int)in.status();
    }
    in.setByteOrder(QDataStream::LittleEndian);
    in.setFloatingPointPrecision(QDataStream::SinglePrecision);
    quint32 ntriangles;
    quint16 control_bytes;
    Facet facet;
    file.seek(80);
    in >> ntriangles;
    for (quint32 k = 0; k < ntriangles;k++) {
        float dat[12];
        for(int i=0;i<12;i++){
            file.seek(84+k*50+i*4);
            in >> dat[i];
        }
        file.seek(84+k*50+48);
        in >> control_bytes;
        //facet.normal<<dat[0],dat[1],dat[2];法相自己计算。
        facet.vertex1<<dat[3],dat[4],dat[5];
        facet.vertex2<<dat[6],dat[7],dat[8];
        facet.vertex3<<dat[9],dat[10],dat[11];
        //normal=(p3-p2)X(p1-p2)| d=-p1*normal;
        Eigen::Vector3f dirct(facet.vertex3-facet.vertex2);
        Eigen::Vector3f normVect(dirct.cross(facet.vertex1-facet.vertex2));
        normVect.normalize();
        float d=-facet.vertex1.dot(normVect);
        facet.plane<<normVect,d;
//        COBOT_LOG.debug()<<"facet ID: "<<k;
//        COBOT_LOG.debug()<<"facet.plane: "<<facet.plane(0)<<","<<facet.plane(1)<<","<<facet.plane(2)<<","<<facet.plane(3);
//        COBOT_LOG.debug()<<"facet.vertex1: "<<facet.vertex1(0)<<","<<facet.vertex1(1)<<","<<facet.vertex1(2);
//        COBOT_LOG.debug()<<"facet.vertex2: "<<facet.vertex2(0)<<","<<facet.vertex2(1)<<","<<facet.vertex2(2);
//        COBOT_LOG.debug()<<"facet.vertex3: "<<facet.vertex3(0)<<","<<facet.vertex3(1)<<","<<facet.vertex3(2);
        m_model_data.push_back(facet);

    }
}