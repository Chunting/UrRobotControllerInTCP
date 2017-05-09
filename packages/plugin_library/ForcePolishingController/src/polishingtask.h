#ifndef POLISHINGTASK_H
#define POLISHINGTASK_H

#include <cobotsys_abstract_object.h>
#include <QJSEngine>
#include <frames.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#ifndef COBOT_DEBUG
    #define COBOT_DEBUG false
#endif
/**
 * \note PTD:Polishing Task Description
 * PolishingTask.ptd is a XML Document that file extension name is customized with ptd.
 */



/**
 * \def EPSILON: norm value used to determine whether two vector are approximately equal
 */

#define EPSILON 0.001


/** \brief This struct is used to keep information of line segment with contacted force.
 *  \memberof plane: plane where middle point of LineSegment located
 *  ,coeff[0-2] contains norminal direction information.
 *  \memberof curveType: the type of c
 */

struct LineSegment_Type {
    Eigen::Vector3d startPoint;
    Eigen::Vector3d endPoint;
    Eigen::Vector3d nominalDirection;
    double force;
};

/** \brief a list of trajectory points.
 */
typedef std::vector<KDL::Frame> Trajectory_Type;
/** \brief Enum for representing the different segment types. May be expanded with more types in the future.
 */

enum Curve_Type {
    Invalid = -1,
    LINE,
    ARC
};

/** \brief This struct represent polishing segment on the product.
 * \memberof curveType : segment's curve type.
 * \memberof startPoint : start point of polishing curve segment.
 * \memberof endPoint : end point of polishing curve segment.
 * \memberof startTangentDirection: the tangent vector at start point on the arc.
 *
 *      if the type of curve is ARC, dirct is needed to let the arc's equation be uniquely determined.
 *      if the type of curve is LIN, dirct parameter ignored.
 *
 * \memberof startForce: the force at start point
 * \memberof endForce: the force at end point
 *
 *      The force in the entire path linearly changes from start point force to end point force.
 *      If the contact force at start and end point are equal, it means that constant force control is to be performed.
 *      At present, we only consider the situation of constant force control on basic path, the future can also be expanded.
*/

struct Segment_Type {
    Curve_Type curveType;
    Eigen::Vector3d startPoint;
    Eigen::Vector3d endPoint;
    Eigen::Vector3d startTangentDirection;
    double startForce;
    double endForce;

};


/** \brief This struct represent an arc function.
 *
 * this data structure is more convenient for parsing staight line segment list.
 *
 * \memberof normalDirection : normal vector of the plane where the arc is located.
 * \memberof centerPoint: center point coordinate.
 * \memberof startPoint: the start point of arc.
 * \memberof radius:  the radius of circle.
 * \memberof angle: the angle from start point to end point.
 * \memberof status: Valid status.
 * this information is parsed from a Segment_Type data that it's Curve_Type is ARC.
 * so the information provided may be invalid. Status is false, when valid; Status is true, when invalid.
*/

    struct ArcFunction_Type {
        Eigen::Vector3d normalDirection;
        Eigen::Vector3d centerPoint;
        Eigen::Vector3d startPoint;
        double radius;
        double angle;
        bool status;
    };

/** \brief a continuous polishing segment. a list of polishing segment construct a while polishing path
 *  \memberof the type of curve witch polishingSegment comes from;
 */
    struct PolishingSegment_Type {
        std::vector<LineSegment_Type>  data;
        Curve_Type curve_type;
        ArcFunction_Type arc;
    };

/** \brief CPolishingTask is used to parse polishing task description and provide access to those information.
 * \note the name CPolishingTask is more suitable, I change the name of this class from CPolishingPath to CPolishingTask.
 *
 *      A complete description of the polishing task, which mainly includes the following:
 *          1.workshop description include robot and polisher description;
 *          2.product description with polishing path description.
 *      polishing path is described as a list of polishing segment with kinds of curve type
 *
 * \memberof m_lineSegmentList: polishing path described  as line segment List.
 * \memberof m_cartesianTrajectory: polishing point list under the EE frame.
 * \memberof m_origFrame: origin frame of product in workshop coordinate system.
 * \memberof m_segmentPrecision:
 *      the precision used to divide the curve polishing segment to small line polishing segment.
 *      it represent the maximun length of line segment.
 *      when the value of m_segmentPrecision is less than zero,
 *      it means the precision will be automatically determined.
 * \memberof m_LinePrecision: line segment precision.
 * \memberof m_ArcPrecision: arc segment precision.
 * \memberof m_ptdPath: the file path and name of .ptd file
 * \memberof m_productModelPath: the path of product model.
 * \memberof m_JSEngine: JavaScript Engine.
 * \memberof m_polisher_frames: the frame of contact point on polishing machine.
*/


class CPolishingTask {
public:
    CPolishingTask(double linePrecision = -1.0 ,double arcPrecision = -1.0);

    /** \param filename: ptd file path with file name.
     */
    CPolishingTask(const std::string &fileName, double segmentPrecision = -1.0 ,double arcPrecision = -1.0);

    /** \brief parse Polishing Task Description (PTD) file.
    \param segmentPrecision maximum length of polishing segment.
    \return if parsed success, then return true, otherwise, return false.
    \note it assume that ptd fils path is already recorded in m_ptdPath
    */
    bool parsePTD(double segmentPrecision = -1.0 ,double arcPrecision = -1.0);

    /** \brief parse Polishing Task Description (PTD) file.
    \param ptdName ptd file full path name.
    \param segmentPrecision maximum length of polishing segment.
    \return if parsed success, then return true, otherwise, return false.
    \note it assumes that ptd fils path is already given to m_ptdPath.
    */
    bool parsePTD(const std::string &ptdName, double segmentPrecision = -1.0 ,double arcPrecision = -1.0);

    /** \brief get Line segment list on the product parsed from polishing task
     * It should be called after parsePTD function is called.
    */
    const std::vector<PolishingSegment_Type>& getPolishingSegments(){ return m_polishingSegments;}

    /** \brief get polishing point list under the EE frame.
     * It should be called after parsePTD function is called.
    */
    const std::vector<Trajectory_Type>& getCartesianTrajectory(){return m_cartesianTrajectory;}

    /** \brief parse robot origin Frame in the workshop coordinate system
     */
    const KDL::Frame& getRobotFrame(){return m_RobotFrame;}


    /** \brief parse polishing machine origin Frame in the workshop coordinate system
     */
    const std::vector<KDL::Frame>& getPolisherFrames(){return m_polisher_frames;}

    /** \brief set product frame in the EE coordinate system
     */
    void setEEFrame(KDL::Frame ee_frame){m_ee_frame=ee_frame;}

    /** \brief Version of CPolishingTask class with int type
     */
    static const int VERSION = 104;

    /** \brief Version of CPolishingTask class with std::string type
     */
    static const std::string VERSION_STR;

    /** \brief write the parsed information(polishing path, orgin frame etc) to XML file.
     * this description file is different from PTD file, this description file is polishing execution oriented
     * and PTD file is polishing task oriented.
     */
    bool write2XML(std::string path);

    /** \brief parse an arc data to ArcFunction_Type type
     *
     */
    ArcFunction_Type arcParser(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint, Eigen::Vector3d startTangentDirection);

protected:

    /** \brief parse a data of Segment_Type type to a list of LineSegment_Type type and push parsed data to m_lineSegmentList
    */
    bool push2LineSegmentList(Segment_Type segment);

    /** \brief parse a vector string such as "0.1 0.3 0.8" to Eigen::Vector3d type.
     */
    Eigen::Vector3d vectorParser(const QString vectorString);

    /** \brief convert a xyz String and rpy String to KDL::Frame.
     *  \example:
     *      (xyzString:"0.1 0.3 0.8" , rpyString:"0 0 ${pi/2}") -> Eigen::Vector3d
     *  \note: the parser support javaSript, it can evaluate the value of expression contained in ${}.
    */
    KDL::Frame FrameParser(const QString xyzString, const QString rpyString);

    /** \brief parse frame from point and norminal direction.
    *
    */
    KDL::Frame FrameParser(Eigen::Vector3d point,Eigen::Vector3d axis);

    /** \brief update norminal dirction list of m_lineSegmentList.
     *  \note: the parser support javaSript, it can evaluate the value of expression contained in ${}.
    */
    bool updateNorminalDirction();

    /** \brief Update cartesian trajectory list
     *
     */
     bool updateTrajectory();//Under the EE Frame


    /** \brief convert a vector String and double vector.
    */
    std::vector<double> getNumList(const QString vectorString);

    /** \brief Verify whether the *.ptd file is valid.
    */
    bool verifyPTD();

private:
    std::vector<PolishingSegment_Type> m_polishingSegments;
    std::vector<Trajectory_Type> m_cartesianTrajectory;// Under the EE Frame
    KDL::Frame m_RobotFrame;
    KDL::Frame m_ee_frame;
    std::vector<KDL::Frame> m_polisher_frames;
    double m_segmentPrecision;
    double m_LinePrecision;
    double m_ArcPrecision;
    QString m_ptdPath;
    std::string m_productModelPath;
    QJSEngine m_JSEngine;
};
#endif // POLISHINGTASK_H
