// https://www.youtube.com/watch?v=AMgqKux1yV8
/*
Alipiev O. (1988), Geometry and Forming of Epiand Hypo-Cycloidal Toothed Wheels in Modified CycloTransmission, Ph.D. Dissertation, Ruse.  Ruse, pp.1-36 (in Bulgarian). 

*/

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include <math.h>
#include <sstream>
#include <algorithm>
#include "HypocycloidCam.h"
#include "HypocycloidConfig.h"
#define maxpoints 10000		// the maximum amount of points on the CAM
#define minlobes 5			// the minimum amount of lobes
#define maxlobes 200		// the maximum amount of lobes
#define minincrement 11		// a lobe must be divided into minimum x sections
#define maxincrement 1001	// a lobe must be divided into maximum x sections
#define minshaftCount 2		// the minimum amount of shafts
#define maxshaftcount 10	// the maximum amount of shafts

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

Ptr<Application> _app;
Ptr<UserInterface> _ui;
Ptr<HypocycloidCam> cam1;
Ptr<HypocycloidCam> cam2;
Ptr<HypocycloidConfig> config;

namespace {

	const std::string adskHypoDesignCmdDefID = "adskHypoDesignCmdDef";
	const std::string camTabID = "camTab";
	const std::string camteethID = "camteeth";
	const std::string modulusID = "modulus";
	const std::string incrementID = "increment";
	const std::string xID = "X";
	const std::string txtID = "txt";
	const std::string pinHolderFilletID = "pinHolderFillet";
	const std::string shaftHolderFilletID = "shaftHolderFillet";
	const std::string geomTabID = "geomTab";
	const std::string camThicknessID = "camThicknessID";
	const std::string shaftCountID = "shaftCountID";
	const std::string camNeedleRollerOuterDiameterID = "camNeedleRollerOuterDiameterID";
	//const std::string camNeedleRollerInnerDiameterID = "camNeedleRollerInnerDiameterID";
	const std::string housingRollerOuterDiameterID = "housingRollerOuterDiameterID";
	const std::string housingRollerInnerDiameterID = "housingRollerInnerDiameterID";
	const std::string HYPOCYCLOID_CAM = "Hypocycloid CAM";
	
	

	//globals
	Ptr<ValueCommandInput> modulusCmd;
	Ptr<IntegerSpinnerCommandInput> camteethCmd;
	Ptr<IntegerSpinnerCommandInput> incrementCmd;
	Ptr<IntegerSpinnerCommandInput> shaftCountCmd;
	Ptr<ValueCommandInput> xCmd;
	Ptr<ValueCommandInput> pinHolderFilletCmd;
	Ptr<ValueCommandInput> shaftHolderFilletCmd;
	Ptr<TextBoxCommandInput> camTxtCmd;
	Ptr<ValueCommandInput> thicknessCmd;
	Ptr<ValueCommandInput> camNeedleRollerOuterDiameterCmd;
	//Ptr<ValueCommandInput> camNeedleRollerInnerDiameterCmd;
	Ptr<ValueCommandInput> housingRollerOuterDiameterCmd;
	Ptr<ValueCommandInput> housingRollerInnerDiameterCmd;
	Ptr<TextBoxCommandInput> geomTxtCmd;
	

	double aw = 0;	// center distance, see figure 7
	double beta = 0;	// contact angle
	double d = 0;   //correction coefficient of rcs
	double d2 = 0;	// pitch circle diameter
	double dc = 0;	// pin diameter
	double d1 = 0;	// diameter of the pitch circle
	double da1 = 0;	// diameter of addendum circle
	double da2 = 0;	// diameter of addendum circle
	double df1 = 0;	// diameter of dedendum circle
	double dTf1 = 0;// diameter of theoretical dedendum circle
	double dta1 = 0;// diameter of theoretical addendum circle
	double dw1 = 0;	// epicycloidal wheel diameter
	double dw2 = 0;	// pins wheel diameter
	double e0 = 0;	// eccentricity of the generating circle
	double e = 0;	// eccentricity of the profile cutting instrument
	double en = 0;	// beneficial overlap ratio OR Optimal contact ratio
	double g = 0;	// angle of motion transfer
	double h1 = 0;	// teeth depth
	int i = 0;
	double l = 0;	// coefficient of non-centrodity
	double m = 0;	// module
	
	double p = 0;	// pitch of the profile cutting instrument
	double r0 = 0;	// radius of epicycle [mm]
	double r1 = 0;	// pitch circle radius
	double r2 = 0;	// 
	double rc = 0;	// pins radius
	double ra2 = 0; // radius of addendum circle
	
	double rcs = 1;	// coefficient of the circle radius, equals to 1 according to Alipiev 1988
	double rf1 = 0; // radius of the working profile dedendum ccircle
	double rTf1 = 0;	// radius of theoretical dedendum circle
	double rw0 = 0;	// radius of the instrument, see 2.1.3
	double rw1 = 0;	// radius of the centrode of the epicycloid
	double rw2 = 0; // radius of the pins circle
	//double t=0;		// anlge of rotation of the generating wheel
	double X = 0;	// modification of the profile cutting instrument
	double x = 0;	// coefficient of modification
	double xmin = 0;
	double xmax = 1;
	double ya = 0;	// allowable angle of transmitting
	int z1 = 0;
	int z2 = 0;
	const double PI = atan(1) * 4; //3.141592653589793238463;
	double camthickness = 0;
	int shaftCount = 0;
	double camNeedleRollerOuterDiam = 0;
	double camNeedleRollerInnerDiam = 0;
	double housingRollerOuterDiam = 0;
	double housingRollerInnerDiam = 0;
	double camHoleRadius = 0;
	double camShaftHolesRadius = 0; //circle on which the roller cylindrical cutouts are
	double ymax = 0;	// 
	double pinHolderFillet = 0;
	double shaftHolderFillet = 0;


	void updateValues() {
		// read the values from the gui
		x = xCmd->value();			// from the gui
		m = modulusCmd->value();	// from the gui
		z1 = camteethCmd->value();	// from the gui
		pinHolderFillet = pinHolderFilletCmd->value(); // from the gui
		shaftHolderFillet = shaftHolderFilletCmd->value(); // from the gui

		camthickness = thicknessCmd->value(); // from the gui
		p = m*PI;					// pitch circle, this is where the pins are located equation 2.1.2
		l = 1 / (1 - x);			// equation 2.1.59
		z2 = z1 + 1;				// chapter 2.1.2
		h1 = m*(1 - x);				// 2.1.24
		d2 = m*z2;					// 
		beta = asin(1 - x);
		rcs = 1;					// Alipiev 1988
		rc = rcs*m;					// 2.1.3
		e = m / 2;					// 2.1.4
		X = e - e0;					// 2.1.5
		r1 = m*z1 / 2;				// 2.1.10
		d1 = m*z1;					// 2.1.11
		rTf1 = r1 + X;				// 2.1.12
		dTf1 = m*(z1 + x);			// 2.1.13
		dta1 = m*(z1 + 2 - x);		// 2.1.16
		rf1 = rTf1 - rc;			// 2.1.17
		df1 = m*(z1 + x - 2 * rcs);	// 2.1.19
		da1 = m*(z1 + 2 - x - 2 * rcs);		// 2.1.22
		dw1 = m*z1*(1 - x);
		dw2 = m*z2*(1 - x);
		aw = m / 2 * (1 - x);		// 2.1.43 center distance is equal to the eccentricity
		rw1 = m*z1 / 2 * (1 - x);	// 2.1.47
		rw2 = m*z2 / 2 * (1 - x);	// 2.1.48
		r2 = m*z2 / 2;				// 2.1.49
		dc = 2 * rcs*m;				// 2.1.50
		ra2 = r2 - rc;				// 2.1.51
		da2 = m*(z2 - 2 * rcs);		// 2.1.53
		ya = 0;						// tbd
		ymax = asin(1 - x);			// 2.1.68
		en = z2 / PI*acos(sin(ya / (1 - x)));							// equation 2.1.77
		xmin = 1 - sqrt(1 - 4 * pow(z1 + 2, 3)*pow(rc, 2) / (27 * z1*pow(z1 + 1, 2))); // equation 2.1.78

		// now the geometric readings
		i = incrementCmd->value();	// from the gui
		shaftCount = shaftCountCmd->value();

		housingRollerOuterDiam = housingRollerOuterDiameterCmd->value();
		housingRollerInnerDiam = housingRollerInnerDiameterCmd->value();
		camNeedleRollerOuterDiam = camNeedleRollerOuterDiameterCmd->value();
		camNeedleRollerInnerDiam = housingRollerInnerDiam + 2 * aw; // inner needle diameter must be 2 x eccentricity larger than housing roller inner diameter
		camHoleRadius = rf1 - camthickness * 2 - camNeedleRollerOuterDiam;
		camShaftHolesRadius = rf1 - camthickness - camNeedleRollerOuterDiam / 2;
	}

	class HypoCmdExecuteEventHandler : public CommandEventHandler
	{
	private:
		Ptr<CircularPatternFeature> makeCircularPattern(Ptr<Occurrence> component, Ptr<Component>root) {
			Ptr<ObjectCollection>inputEntites = adsk::core::ObjectCollection::create();
			if (!inputEntites) return NULL;
			inputEntites->add(component);
			Ptr<CircularPatternFeatures>circularFeats = root->features()->circularPatternFeatures();
			if (!circularFeats) return NULL;
			Ptr<CircularPatternFeatureInput>circularFeatInput = circularFeats->createInput(inputEntites, root->yConstructionAxis());
			if (!circularFeatInput) return NULL;
			circularFeatInput->quantity(ValueInput::createByReal(shaftCount));
			circularFeatInput->totalAngle(ValueInput::createByString("360 deg"));
			Ptr<CircularPatternFeature>circularFeat = circularFeats->add(circularFeatInput);
			if (!circularFeat) return NULL;
			return circularFeat;
		}
		
	public:
		void notify(const Ptr<CommandEventArgs>& eventArgs) override
		{

			Ptr<Design> design = _app->activeProduct();
			Ptr<Command> cmd = eventArgs->command();
			Ptr<CommandInputs> inputs = cmd->commandInputs();

			config = new HypocycloidConfig();
			cam1 = new HypocycloidCam();
			cam2 = new HypocycloidCam();

			// Save the current values as attributes.
			Ptr<Design> des = _app->activeProduct();
			Ptr<Attributes> attribs = des->attributes();

			// create a Hypo gear component
			Ptr<Component> root = design->rootComponent();


			// create a new component for the CAM
			Ptr<Occurrences>occurences = root->occurrences();
			Ptr<Occurrence> camOccurrence = occurences->addNewComponent(Matrix3D::create());
			camOccurrence->activate();
			Ptr<Component>cam = camOccurrence->component();
			cam->name(HYPOCYCLOID_CAM);
			cam->description("This is the hypocycloidal cam");

			// Create a new sketch
			Ptr<Sketches> camsketches = cam->sketches();
			Ptr<Sketch> piesketch = camsketches->add(cam->xZConstructionPlane());
			piesketch->name("hypocycloid cam pie");
			piesketch->isVisible(true);

			updateValues();

			// create a point collection to form the spline
			Ptr<ObjectCollection> points = ObjectCollection::create();
			Ptr<Point3D>origin = Point3D::create(0, 0, 0);
			Ptr<Point3D>begin, end;
			double maxangle = (2 * PI) / z1;
			double increment = maxangle / (double)i;
			int c = 0;
			for (int j = 0; j <= i*z1; j++) { // iterate through for loops only with integers!!!
				c++;
				double t = j*increment;
				double xt = -aw + m / 2 * (z2*sin(t) - (1 - x)*sin(z2*t) + (2 * ((1 - x)*sin(z2*t) - sin(t))) /
					sqrt(1 - 2 * (1 - x)*cos(z1*t) + pow((x - 1), 2)));
				double yt = m / 2 * (z2*cos(t) - (1 - x)*cos(z2*t) + (2 * ((1 - x)*cos(z2*t) - cos(t))) /
					sqrt(1 - 2 * (1 - x)*cos(z1*t) + pow((x - 1), 2)));
				if (t == 0) {
					begin = Point3D::create(xt, yt, 0);
				}
				else {
					end = Point3D::create(xt, yt, 0);
				}
				points->add(Point3D::create(xt, yt, 0));
			}

			Ptr<SketchCurves> piesketchCurves = piesketch->sketchCurves();
			Ptr<SketchFittedSplines> piesplines = piesketchCurves->sketchFittedSplines();
							
			Ptr<SketchFittedSpline>spline = piesplines->add(points);
			Ptr<SketchLines> piesketchLines = piesketchCurves->sketchLines();

			adsk::doEvents();

			// now extrude the profile
			Ptr<ExtrudeFeatures> extrudes = cam->features()->extrudeFeatures();
			Ptr<ExtrudeFeatureInput> extrudeFeature = extrudes->createInput(piesketch->profiles()->item(0), FeatureOperations::NewBodyFeatureOperation);
			extrudeFeature->setDistanceExtent(false, ValueInput::createByReal(camthickness));
			Ptr<ExtrudeFeature> extrudedCam = extrudes->add(extrudeFeature);
			extrudedCam->name("Cam extrusion");

			
			// here come the cam roller bearings holes
			Ptr<Sketches> shaftsholeketches = cam->sketches();
			Ptr<Sketch> shaftsholesketch = shaftsholeketches->add(cam->xZConstructionPlane());
			shaftsholesketch->name("Shaft holes sketch");
			Ptr<SketchCircles>shaftsholecircles = shaftsholesketch->sketchCurves()->sketchCircles();
			Ptr<ObjectCollection>shaftHoles = ObjectCollection::create();
			for (int j = 0; j < shaftCount; j++) {
				double phi = (2 * PI) / shaftCount*j;
				Ptr<Point3D> point = Point3D::create(camShaftHolesRadius*cos(phi) - aw, camShaftHolesRadius*sin(phi), 0);
				shaftHoles->add(point);
				shaftsholecircles->addByCenterRadius(point, camNeedleRollerOuterDiam / 2);
				Ptr<ExtrudeFeatures> shaftsholeextrudes = cam->features()->extrudeFeatures();
				Ptr<ExtrudeFeatureInput> shaftsholeextrudeFeature = shaftsholeextrudes->createInput(shaftsholesketch->profiles()->item(j), FeatureOperations::CutFeatureOperation);
				shaftsholeextrudeFeature->setDistanceExtent(false, ValueInput::createByReal(camthickness));
				Ptr<ExtrudeFeature> shaftshole = shaftsholeextrudes->add(shaftsholeextrudeFeature);
			}

			// now comes the main hole in the cam
			shaftsholecircles->addByCenterRadius(Point3D::create(-aw, 0, 0), camHoleRadius);
			Ptr<ExtrudeFeatures> shaftsholeextrudes = cam->features()->extrudeFeatures();

			Ptr<ExtrudeFeatureInput> shaftsholeextrudeFeature = shaftsholeextrudes->createInput(shaftsholesketch->profiles()->item(shaftCount), FeatureOperations::CutFeatureOperation);
			shaftsholeextrudeFeature->setDistanceExtent(false, ValueInput::createByReal(camthickness));
			Ptr<ExtrudeFeature> shaftshole = shaftsholeextrudes->add(shaftsholeextrudeFeature);
			shaftshole->name("Shaft hole");

			// now come the recess sketch
			Ptr<Sketches> camRecessSketches = cam->sketches();
			Ptr<Sketch> recessSketch = camRecessSketches->add(cam->xZConstructionPlane());
			recessSketch->name("Recess sketch");
			Ptr<SketchCircles>recessCircles = recessSketch->sketchCurves()->sketchCircles();
			
			for (int k = 0; k < shaftCount; k++) {
				Ptr<SketchCircle>circle= recessCircles->addByCenterRadius(shaftHoles->item(k), camNeedleRollerOuterDiam / 2 + 1);
				//recessCircles->addByCenterRadius(shaftHoles->item(k), camNeedleRollerOuterDiam / 2);
				//intersects->add(circle);
			}
			


			Ptr<SketchCircle>innercircle = recessCircles->addByCenterRadius(Point3D::create(-aw, 0, 0), camShaftHolesRadius - (camNeedleRollerOuterDiam / 2));
			Ptr<SketchCircle>outercircle = recessCircles->addByCenterRadius(Point3D::create(-aw, 0, 0), camShaftHolesRadius + (camNeedleRollerOuterDiam / 2));
			Ptr<SketchLines> recessLines = recessSketch->sketchCurves()->sketchLines();
			
			Ptr<Profiles> profiles = recessSketch->profiles();
			int pcount = profiles->count();
			int areas[5] = { 0,0,0,0,0 };
			for (int j = 0; j < pcount - 1; j++) {
				Ptr<Profile> profile = profiles->item(j);
				int area = (int)(profile->areaProperties()->area() * 100000);
				bool skipped = false;
				for (int k = 0; k < 5; k++) {
					if (areas[k] == area) {
						skipped = true;
						break; // skip
					}
				}
				if (!skipped) {
					for (int k = 0; k < 5; k++) {
						if (areas[k] == 0) {
							areas[k] = area;
							break;
						}
					}
				}
			}
			std::sort(areas, areas + 5);
			Ptr<ObjectCollection> shaftshole2Profiles = ObjectCollection::create();
			for (int j = 0; j < pcount; j++) {
				Ptr<Profile> profile = profiles->item(j);
				int area = (int)(profile->areaProperties()->area() * 100000);
				if ( area == areas[4])
				{
					shaftshole2Profiles->add(profile);
				}
			}

			Ptr<ExtrudeFeatures> shaftshole2extrudes = cam->features()->extrudeFeatures();
			Ptr<ExtrudeFeatureInput> holesextrudeFeature = shaftshole2extrudes->createInput(shaftshole2Profiles,
				FeatureOperations::CutFeatureOperation);
			holesextrudeFeature->setDistanceExtent(false, ValueInput::createByReal(camthickness));
			Ptr<ExtrudeFeature> extrudedCam2 = extrudes->add(holesextrudeFeature);
			extrudedCam2->name("Cam holes extrusion");
			Ptr<BRepBody>shaftHoles2Body = extrudedCam2->bodies()->item(0);
			// add the internal fillets
			Ptr<BRepEdges>shaftHoles2Edges = shaftHoles2Body->edges();
			size_t shaftHoles2EdgeCount = shaftHoles2Edges->count();
			Ptr<ObjectCollection>shaftHoles2FilletEdges = ObjectCollection::create();
			for (int k = 0; k < shaftHoles2EdgeCount; k++) {
				Ptr<BRepEdge>edge = shaftHoles2Edges->item(k);
				const char* otype = edge->objectType();
				Ptr<BRepVertex>a = edge->startVertex();
				Ptr<BRepVertex>b = edge->endVertex();
				double dist = round(a->geometry()->distanceTo(b->geometry()) * 100);
				if (dist == round(camthickness * 100)) {
					shaftHoles2FilletEdges->add(edge);
				}
			}
			Ptr<FilletFeatures> shaftHoles2Fillets = root->features()->filletFeatures();
			Ptr<FilletFeatureInput> shaftHoles2Input = shaftHoles2Fillets->createInput();
			shaftHoles2Input->addConstantRadiusEdgeSet(shaftHoles2FilletEdges, ValueInput::createByReal(shaftHolderFillet), true);
			shaftHoles2Input->isG2(false);
			shaftHoles2Input->isRollingBallCorner(true);
			Ptr<FilletFeature> shaftHoles2Fillet = shaftHoles2Fillets->add(shaftHoles2Input);
			shaftHoles2Fillet->name("Shafts hole fillets");
			
			// let's draw some more circles
			Ptr<Sketches> helpersketches = cam->sketches();
			Ptr<Sketch> helperSketch = helpersketches->add(cam->xZConstructionPlane());
			helperSketch->name("Helper sketch");
			helperSketch->isVisible(true);
			Ptr<SketchCircles>helperCircles = helperSketch->sketchCurves()->sketchCircles();
			Ptr<SketchCircle>addendum = helperCircles->addByCenterRadius(Point3D::create(-aw, 0, 0), ra2);
			addendum->isConstruction(true);
			Ptr<SketchCircle>dedendum = helperCircles->addByCenterRadius(Point3D::create(-aw, 0, 0), rf1);
			dedendum->isConstruction(true);

			
			// lets draw the housing base
			Ptr<Occurrence> housingComponent = occurences->addNewComponent(Matrix3D::create()); // the occurences can be reused
			housingComponent->activate();
			housingComponent->isGrounded(true);
			Ptr<Component>housing = housingComponent->component();
			housing->name("Hypocycloid housing component");
			Ptr<Sketches> housingsketches = housing->sketches();
			Ptr<Sketch> housingsketch = housingsketches->add(housing->xZConstructionPlane());
			housingsketch->name("Shaft bottom sketch");
			Ptr<SketchCircles>housingCircles = housingsketch->sketchCurves()->sketchCircles();
			Ptr<SketchCircle>circumference = housingCircles->addByCenterRadius(Point3D::create(0, 0, 0), 2 * camShaftHolesRadius);// this is maybe a little too large
			Ptr<ExtrudeFeature> housingExtr = housing->features()->extrudeFeatures()->addSimple(housingsketch->profiles()->item(0),
				ValueInput::createByReal(-camthickness), FeatureOperations::NewBodyFeatureOperation);
			housingExtr->name("Housing extrusion");
			// here comes the main hole in the housing
			Ptr<SketchCircle> housingHole = housingCircles->addByCenterRadius(Point3D::create(0, 0, 0), camHoleRadius);
			Ptr<ExtrudeFeature> housingHoleExtr = housing->features()->extrudeFeatures()->addSimple(housingsketch->profiles()->item(1),
				ValueInput::createByReal(-camthickness), FeatureOperations::CutFeatureOperation);
			housingHoleExtr->name("Housing hole extrusion");
			// and here come the shaft holes in the housing
			for (int j = 0; j < shaftCount; j++) {
				double phi = (2 * PI) / shaftCount*j;
				Ptr<SketchCircle>circle = housingCircles->addByCenterRadius(Point3D::create(camShaftHolesRadius*cos(phi), camShaftHolesRadius*sin(phi), 0), housingRollerOuterDiam / 2);// this is maybe a little too large

				Ptr<ExtrudeFeature> housingHoleExtr = housing->features()->extrudeFeatures()->addSimple(housingsketch->profiles()->item(2 + j),
					ValueInput::createByReal(-camthickness), FeatureOperations::CutFeatureOperation);
				housingHoleExtr->name("ShaftHole" + std::to_string(j + 1));
			}

			// now add the 1st roller bearings according to their size
			Ptr<Occurrence>housingRollerOccurrence = occurences->addNewComponent(Matrix3D::create()); // the occurences can be reused
			housingRollerOccurrence->activate();
			Ptr<Component>housingRoller = housingRollerOccurrence->component();
			//housingRollerOccurrence->name(HOUSING_ROLLER);
			housingRoller->name("Housing roller");
			Ptr<Sketch> housingRollerSketch = housingRoller->sketches()->add(housingRoller->xZConstructionPlane());
			housingRollerSketch->name("Housing roller sketch");
			Ptr<SketchCircles>housingRollerCircles = housingRollerSketch->sketchCurves()->sketchCircles();
			Ptr<SketchCircle>rollerouter = housingRollerCircles->addByCenterRadius(Point3D::create(camShaftHolesRadius, 0, 0), housingRollerOuterDiam / 2);
			Ptr<SketchCircle>rollerinner = housingRollerCircles->addByCenterRadius(Point3D::create(camShaftHolesRadius, 0, 0), housingRollerInnerDiam / 2);
			Ptr<ExtrudeFeature> housinRollerFeature = housingRoller->features()->extrudeFeatures()->addSimple(housingRollerSketch->profiles()->item(0),
				ValueInput::createByReal(-camthickness), FeatureOperations::NewBodyFeatureOperation);
			housinRollerFeature->name("Housing roller extrusion");

			// now add the 1st needle roller bearings according to their size
			Ptr<Occurrence>camNeedleRollerOccurrence = occurences->addNewComponent(Matrix3D::create()); // the occurences can be reused
			camNeedleRollerOccurrence->activate();
			Ptr<Component>camNeedleRoller = camNeedleRollerOccurrence->component();
			camNeedleRoller->name("Cam needle roller");
			Ptr<Sketch> camNeedleRollerSketch = camNeedleRoller->sketches()->add(camNeedleRoller->xZConstructionPlane());
			camNeedleRollerSketch->name("Cam needle roller sketch");
			Ptr<SketchCircles>camNeedleRollerCircles = camNeedleRollerSketch->sketchCurves()->sketchCircles();
			Ptr<SketchCircle>camNeedleOuter = camNeedleRollerCircles->addByCenterRadius(Point3D::create(camShaftHolesRadius - aw, 0, 0), camNeedleRollerOuterDiam / 2);
			Ptr<SketchCircle>camNeedleInner = camNeedleRollerCircles->addByCenterRadius(Point3D::create(camShaftHolesRadius - aw, 0, 0), camNeedleRollerInnerDiam / 2);
			Ptr<ExtrudeFeature>camNeedleRollerExtrusion = camNeedleRoller->features()->extrudeFeatures()->addSimple(camNeedleRollerSketch->profiles()->item(0),
				ValueInput::createByReal(camthickness), FeatureOperations::NewBodyFeatureOperation);
			camNeedleRollerExtrusion->name("Cam needle roller extrusion");

			// now we start to create the shaft
			Ptr<Occurrence>shaftOccurrence = occurences->addNewComponent(Matrix3D::create());
			shaftOccurrence->activate();
			Ptr<Component>shaft = shaftOccurrence->component();
			shaft->name("hypocycloid shaft");
			Ptr<Sketches> shaftsketches = shaft->sketches();
			Ptr<Sketch> shaftsketch = shaftsketches->add(shaft->xZConstructionPlane());
			shaftsketch->name("shaft bottom sketch");
			Ptr<SketchCircles>shaftCircles = shaftsketch->sketchCurves()->sketchCircles();
			Ptr<SketchCircle>shaftToHousing = shaftCircles->addByCenterRadius(Point3D::create(camShaftHolesRadius, 0, 0), housingRollerInnerDiam / 2);
			shaft->features()->extrudeFeatures()->addSimple(shaftsketch->profiles()->item(0),
				ValueInput::createByReal(-camthickness), FeatureOperations::NewBodyFeatureOperation);
			Ptr<SketchCircle>shaftToCam = shaftCircles->addByCenterRadius(Point3D::create(camShaftHolesRadius - aw, 0, 0), camNeedleRollerInnerDiam / 2);
			Ptr<ExtrudeFeatures>features = shaft->features()->extrudeFeatures();
			features->addSimple(shaftsketch->profiles()->item(0),
				ValueInput::createByReal(camthickness), FeatureOperations::JoinFeatureOperation);
			features->addSimple(shaftsketch->profiles()->item(1),
				ValueInput::createByReal(camthickness), FeatureOperations::JoinFeatureOperation);
			Ptr<ConstructionPlanes> shaftPlanes = shaft->constructionPlanes();	// add a construction plane here to sketch on
			Ptr<ConstructionPlaneInput> shaftPlaneLayer2 = shaftPlanes->createInput();
			shaftPlaneLayer2->setByOffset(shaft->xZConstructionPlane(), ValueInput::createByReal(camthickness));
			Ptr<ConstructionPlane> shaftLayer2Plane = shaftPlanes->add(shaftPlaneLayer2);
			Ptr<Sketch> layer2 = shaftsketches->add(shaftLayer2Plane);
			layer2->name("2nd layer");
			Ptr<SketchCurves> shaftSketchLayer2Curves = layer2->sketchCurves();
			Ptr<SketchCircles> shaftSketchLayer2Circles = shaftSketchLayer2Curves->sketchCircles();
			shaftSketchLayer2Circles->addByCenterRadius(Point3D::create(camShaftHolesRadius + aw, 0, 0), camNeedleRollerInnerDiam / 2);
			shaft->features()->extrudeFeatures()->addSimple(layer2->profiles()->item(0),
				ValueInput::createByReal(camthickness), FeatureOperations::JoinFeatureOperation);
			Ptr<ConstructionPlaneInput> shaftPlaneLayer3 = shaftPlanes->createInput();
			shaftPlaneLayer3->setByOffset(shaft->xZConstructionPlane(), ValueInput::createByReal(camthickness * 2));
			Ptr<ConstructionPlane> shaftLayer3Plane = shaftPlanes->add(shaftPlaneLayer3);
			Ptr<Sketch> layer3 = shaftsketches->add(shaftLayer3Plane);
			layer2->name("3rd layer");
			Ptr<SketchCurves> shaftSketchLayer3Curves = layer3->sketchCurves();
			Ptr<SketchCircles> shaftSketchLayer3Circles = shaftSketchLayer3Curves->sketchCircles();
			shaftSketchLayer3Circles->addByCenterRadius(Point3D::create(camShaftHolesRadius, 0, 0), housingRollerInnerDiam / 2);
			Ptr<ExtrudeFeature>shaftExtruded = shaft->features()->extrudeFeatures()->addSimple(layer3->profiles()->item(0),
				ValueInput::createByReal(camthickness), FeatureOperations::JoinFeatureOperation);
			shaftExtruded->name("Shaft extrusion");

			adsk::doEvents();

			// now let's draw the pins holder
			occurences->item(0)->activate();
			Ptr<Occurrence>pinsHolderOccurrence = occurences->addNewComponent(Matrix3D::create());
			pinsHolderOccurrence->activate();
			Ptr<Component>pHolder = pinsHolderOccurrence->component();
			pHolder->name("Pins holder");

			Ptr<Sketches> pHolderSketches = pHolder->sketches();
			Ptr<Sketch> pHolderSketch = pHolderSketches->add(pHolder->xZConstructionPlane());
			pHolderSketch->name("Pin holder bottom sketch");
			pHolderSketch->isVisible(true);
			Ptr<SketchCircles>pHolderSketchCircles = pHolderSketch->sketchCurves()->sketchCircles();
			Ptr<ExtrudeFeatures> pinsextrudes = pHolder->features()->extrudeFeatures();
			Ptr<ObjectCollection> pinsExtrudeProfiles = ObjectCollection::create();
			for (int j = 0; j < z2; j++) {
				Ptr<Point3D> point = Point3D::create(r2*cos(2 * PI*j / z2), r2*sin(2 * PI*j / z2), 0);
				Ptr<SketchCircle>circle = pHolderSketchCircles->addByCenterRadius(point, dc / 2);
			}
			// creating the outer rim 
			Ptr<SketchCircle>cirle2 = pHolderSketchCircles->addByCenterRadius(Point3D::create(0, 0, 0), r2 + dc / 2);
			Ptr<SketchCircle>cirle3 = pHolderSketchCircles->addByCenterRadius(Point3D::create(0, 0, 0), r2);
			profiles = pHolderSketch->profiles();
			for (int k = 0; k < profiles->count(); k++) {
				
				Ptr<Profile> profile = profiles->item(k);
				Ptr<ProfileLoop> loop = profile->profileLoops()->item(0);
				if(loop->profileCurves()->count()<=5){
					pinsExtrudeProfiles->add(profile);
				}
			}

			Ptr<ExtrudeFeatureInput> pinsextrudeFeature = pinsextrudes->createInput(pinsExtrudeProfiles, FeatureOperations::JoinFeatureOperation);
			pinsextrudeFeature->setDistanceExtent(false, ValueInput::createByReal(camthickness));
			Ptr<ExtrudeFeature> pinsext = pinsextrudes->add(pinsextrudeFeature);
			Ptr<BRepBody>pinsBody = pinsext->bodies()->item(0);
		
			Ptr<BRepEdges>edges = pinsBody->edges();
			
			size_t edgeCount = edges->count();
			Ptr<ObjectCollection>filletEdges = ObjectCollection::create();
			for (int k = 0; k < edgeCount; k++) {
				Ptr<BRepEdge>edge = edges->item(k);
				const char* otype = edge->objectType();
				Ptr<BRepVertex>a = edge->startVertex();
				Ptr<BRepVertex>b = edge->endVertex();
				double dist = round(a->geometry()->distanceTo(b->geometry())*100);
				if (dist == round(camthickness*100)) {
					filletEdges->add(edge);
				}
			}
			// make all the fillets to the pins holder
			Ptr<FilletFeatures> fillets = root->features()->filletFeatures();
			Ptr<FilletFeatureInput> input1 = fillets->createInput();
			input1->addConstantRadiusEdgeSet(filletEdges, ValueInput::createByReal(pinHolderFillet), true);
			input1->isG2(false);
			input1->isRollingBallCorner(true);
			Ptr<FilletFeature> fillet1 = fillets->add(input1);
			Ptr<BRepBody>pinsholderBody = fillet1->bodies()->item(0);
			
			//move the pins holder, but for some reason this doesn't work
			Ptr<MoveFeatures>mfeats = root->features()->moveFeatures();
			Ptr<Matrix3D> transform = adsk::core::Matrix3D::create();
			Ptr<Vector3D> vector = adsk::core::Vector3D::create(0.0, camthickness, 0.0);
			transform->translation(vector);
			Ptr<ObjectCollection> moveFeatureEntities = adsk::core::ObjectCollection::create();
			moveFeatureEntities->add(pinsholderBody);
			size_t p = moveFeatureEntities->count();
			Ptr<MoveFeatureInput> moveFeatureInput = mfeats->createInput(moveFeatureEntities, transform);
									
			// now make a circular pattern of the rollers and the shaft
			//Ptr<CircularPatternFeature> shafts = makeCircularPattern(shaftOccurrence,root);
			//Ptr<CircularPatternFeature> needleRollers= makeCircularPattern(camNeedleRollerOccurrence,root);
			//Ptr<CircularPatternFeature> housingRollers = makeCircularPattern(housingRollerOccurrence,root);


			/*
			Ptr<MoveFeatures>mfeats = root->features()->moveFeatures();
			Ptr<Matrix3D> transform = adsk::core::Matrix3D::create();
			Ptr<Vector3D> vector = adsk::core::Vector3D::create(0.0, 0.0, 10.0);
			transform->translation(vector);
			Ptr<MoveFeatureInput> moveFeatureInput = mfeats->createInput(pinsBody, transform);
			Ptr<MoveFeature> moveFeature = mfeats->add(moveFeatureInput);

			adsk::doEvents();
			
			Ptr<Joints> rootJoints = root->joints();

			// join the first roller
			Ptr<JointGeometry> geoA = JointGeometry::createByProfile(housingRollerSketch->profiles()->item(0), NULL, JointKeyPointTypes::CenterKeyPoint);
			Ptr<JointGeometry> geoB = JointGeometry::createByProfile(housingsketch->profiles()->item(1) , NULL, JointKeyPointTypes::CenterKeyPoint);
			Ptr<JointInput> jointInput = rootJoints->createInput(geoA, geoB);
			jointInput->setAsRigidJointMotion();
			Ptr<Joint> jointShaftBase1 = rootJoints->add(jointInput);
			jointShaftBase1->name("1st roller to housing joint");
			*/
			
			Ptr<Viewport> activeView = _app->activeViewport();
			if (activeView)
				activeView->fit();
		
		}
	} _hypoCmdExecute;

	class HypoCmdInputChangedHandler : public adsk::core::InputChangedEventHandler
	{
	public:
		void notify(const Ptr<InputChangedEventArgs>& eventArgs) override
		{
			
			Ptr<InputChangedEvent> event = eventArgs->firingEvent();
			Ptr<Command> command = event->sender();
			if (!command) // in case it wasn't actually fired by a sender, return
				return;
			Ptr<CommandInput> cmdInput = eventArgs->input();
			if (!cmdInput) // in case the event wasn't fired from an imput command, return
				return;
			updateValues();

			//Ptr<CommandInputs> inputs = command->commandInputs();
			//if (cmdInput->id() == modulusID) {	}
			
			// just output some intermediate results here
			std::string txt = "";
			//txt += "increment	[" + std::to_string(i) + "]\r\n";
			//txt += "xmin	[" + std::to_string(xmin * 10) + "]mm\r\n";
			//txt += "xmax	[" + std::to_string(xmax * 10) + "]mm\r\n";
			txt += "pitch circle [" + std::to_string(d1) + "]cm diameter\r\n";
			txt += "theoret. dedendum circle [" + std::to_string(dTf1) + "]cm diameter\r\n";
			txt += "theoret. addendum circle [" + std::to_string(dta1) + "]cm diameter\r\n";
			txt += "dedendum circle [" + std::to_string(df1) + "]cm diameter\r\n";
			txt += "addendum circle [" + std::to_string(da1) + "]cm diameter\r\n";
			txt += "teeth depth	[" + std::to_string(h1) + "]cm\r\n";
			//txt += "addendum curvature [" + std::to_string(pf) + "]cm radius\r\n";
			//txt += "dedendum curvature [" + std::to_string(pa) + "]cm radius\r\n";
			//txt += "minimal teeth curvature [" + std::to_string(pmin) + "]cm radius\r\n";
			//txt += "lambda [" + std::to_string(lambda) + "]cm\r\n";
			txt += "pins [" + std::to_string(z2) + "]\r\n";
			txt += "pitch circle [" + std::to_string(d2) + "]cm diameter\r\n";
			txt += "pin diameter	[" + std::to_string(dc) + "]cm\r\n";
			txt += "addedndum circle [" + std::to_string(da2) + "]cm diameter\r\n";
			txt += "contact angle [" + std::to_string(beta*360/(2*PI)) + "]deg\r\n";
			txt += "centre distance [" + std::to_string(aw) + "]cm\r\n";
			txt += "epicycloidal wheel [" + std::to_string(dw1) + "]cm diameter\r\n";
			txt += "pins wheel [" + std::to_string(dw2) + "]cm diameter\r\n";
			txt += "non-centeroidity [" + std::to_string(l) + "]cm\r\n";
			//txt += "transmitting movement [" + std::to_string(gammamax) + "] max angle\r\n";
			//txt += "beneficial overlapping [" + std::to_string(taun) + "] angle\r\n";
			//txt += "beneficial overlapping [" + std::to_string(etan) + "] coefficient\r\n";
			txt += "eccentricity	[" + std::to_string(e) + "]cm\r\n";
			txt += "points on cam [" + std::to_string(i*z1) + "] < [" + std::to_string(maxpoints) + "] ok\r\n";
			camTxtCmd->text(txt);


			txt = "";
			txt += "center hole [" + std::to_string(camHoleRadius + 2) + "]cm\r\n";
			txt += "needle inner diameter [" + std::to_string(camNeedleRollerInnerDiam) + "]cm";
			geomTxtCmd->text(txt);
		
		}
	} _hypoCmdInputChanged;


	/*
		This evaluates if the OK button should be enabled or not. 
		In case the user input is invalid, the OK button should be disabled.
	*/
	class HypoCmdValidateInputsEventHandler : public adsk::core::ValidateInputsEventHandler
	{
	public:
		void notify(const Ptr<ValidateInputsEventArgs>& eventArgs) override
		{
			Ptr<Event> firingEvent = eventArgs->firingEvent();
			Ptr<Command> command = firingEvent->sender();
			if (!command) // in case it wasn't actually fired by a sender, return
				return;
			
			// calculate tha values
			updateValues();
			
			//let's assume all is good
			eventArgs->areInputsValid(true);
			

			// now check, if anything is bad
			if (x < xmin) {
				eventArgs->areInputsValid(false);
				std::string txt = "x must be larger than [" + std::to_string(xmin * 10) + "]mm\r\n";
				camTxtCmd->text(txt);
				return;
			}
			if (i % 2 == 0) {
				eventArgs->areInputsValid(false);
				std::string txt = "increments must be an uneven number\r\n";
				camTxtCmd->text(txt);
				return;
			}
			if ((i *z1)>maxpoints) {
				eventArgs->areInputsValid(false);
				std::string txt = "the calculation results on too many points [" + std::to_string(i *z1) + "]\r\n";
				txt += "please reduce z1 or i until the number of points is below [" + std::to_string(maxpoints) + "]\r\n";
				camTxtCmd->text(txt);
				return;
			}
			if (z1<minlobes) {
				eventArgs->areInputsValid(false);
				std::string txt = "the number of lobes must be minimum [" + std::to_string(minlobes) + "]\r\n";
				camTxtCmd->text(txt);
				return;
			}
			if (z1>maxlobes) {
				eventArgs->areInputsValid(false);
				std::string txt = "the number of lobes must be maximum [" + std::to_string(maxlobes) + "]\r\n";
				camTxtCmd->text(txt);
				return;
			}
			if (camNeedleRollerInnerDiam>camNeedleRollerOuterDiam) {
				eventArgs->areInputsValid(false);
				std::string txt = "the inner diameter cannot be larger than the outer diameter\r\n";
				geomTxtCmd->text(txt);
				return;
			}
			

		}
	} _hypoCmdValidateInputs;

	/*
	This is called when the button was clicked and the dialog window must be show
	*/
	class HypoCmdCreatedHandler : public adsk::core::CommandCreatedEventHandler
	{
		void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override
		{
			Ptr<Design> des = _app->activeProduct();
			Ptr<Command> cmd = eventArgs->command();
			cmd->isExecutedWhenPreEmpted(false);
			Ptr<CommandInputs> inputs = cmd->commandInputs();
			// CAM tab
			Ptr<TabCommandInput> tabCamInput = inputs->addTabCommandInput(camTabID, "CAM");
			Ptr<CommandInputs> camTabChildInputs = tabCamInput->children();

			// here we need to create the inputs
			modulusCmd = camTabChildInputs->addValueInput(modulusID, "Modulus", "mm", ValueInput::createByReal(0.5));
			modulusCmd->tooltipDescription("Sets the modulus of the CAM");
			camteethCmd = camTabChildInputs->addIntegerSpinnerCommandInput(camteethID, "CAM teeth", minlobes, maxlobes, 1, 20);
			xCmd = camTabChildInputs->addValueInput(xID, "Coefficient of m", "mm", ValueInput::createByReal(0.1));
			camTxtCmd = camTabChildInputs->addTextBoxCommandInput(txtID, "Results", "content", 25, true);
			incrementCmd = camTabChildInputs->addIntegerSpinnerCommandInput(incrementID, "Increment", minincrement,maxincrement,2, 21);

			// Geometry TAB
			Ptr<TabCommandInput> tabGeomInput = inputs->addTabCommandInput(geomTabID, "Geometry");
			Ptr<CommandInputs> camGeomChildInputs = tabGeomInput->children();
			thicknessCmd = camGeomChildInputs->addValueInput(camThicknessID, "CAM thickness", "mm", ValueInput::createByReal(0.7));
			shaftCountCmd = camGeomChildInputs->addIntegerSpinnerCommandInput(shaftCountID, "shaft count", minshaftCount, maxshaftcount, 1, 2);
			camNeedleRollerOuterDiameterCmd = camGeomChildInputs->addValueInput(camNeedleRollerOuterDiameterID, "CAM needle roller outer diameter", "mm", ValueInput::createByReal(1.4));
			//camNeedleRollerInnerDiameterCmd = camGeomChildInputs->addValueInput(camNeedleRollerInnerDiameterID, "CAM needle roller inner diameter", "mm", ValueInput::createByReal(0.8));
			housingRollerOuterDiameterCmd = camGeomChildInputs->addValueInput(housingRollerOuterDiameterID, "Housing roller outer diameter", "mm", ValueInput::createByReal(2.2));
			housingRollerInnerDiameterCmd = camGeomChildInputs->addValueInput(housingRollerInnerDiameterID, "Housing roller inner diameter", "mm", ValueInput::createByReal(0.8));
			geomTxtCmd = camGeomChildInputs->addTextBoxCommandInput(txtID, "Results", "content", 15, true);
			pinHolderFilletCmd = camGeomChildInputs->addValueInput(pinHolderFilletID, "Pin holder fillet", "mm", ValueInput::createByReal(0.05));
			shaftHolderFilletCmd = camGeomChildInputs->addValueInput(shaftHolderFilletID, "Shaft holder fillet", "mm", ValueInput::createByReal(0.5));
			// start listening to input changes
			cmd->inputChanged()->add(&_hypoCmdInputChanged);
			cmd->validateInputs()->add(&_hypoCmdValidateInputs);
			cmd->execute()->add(&_hypoCmdExecute);
			
		}
	} _hypoCmdCreated;


// try/catch blocks won't work, see http://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ECC0A398-4D89-4776-A054-F7B432F7FCF6
extern "C" XI_EXPORT bool run(const char* context)
	{
	// important update came from Brian Ekins: https://modthemachine.typepad.com/my_weblog/2018/12/prepare-add-in-for-ui-preview.html

		_app = Application::get();
		if (!_app)
			return false;

		_ui = _app->userInterface();
		if (!_ui)
			return false;

		_ui->messageBox("Hello user");

		Ptr<CommandDefinitions> cmdDefs = _ui->commandDefinitions();

		Ptr<CommandDefinition> hypoDesignCmdDef = cmdDefs->itemById(adskHypoDesignCmdDefID);
		if (!hypoDesignCmdDef) {

			//TODO: this returns null
			hypoDesignCmdDef = cmdDefs->addButtonDefinition(adskHypoDesignCmdDefID, "Hypocycloid gears", "Awesome gears ahead", "Resources");
			if (!hypoDesignCmdDef) {
				std::string errorMessage;
				int errorCode = _app->getLastError(&errorMessage);
				if (GenericErrors::Ok != errorCode)
					_ui->messageBox(errorMessage);
			}
		}
		
		Ptr<Workspaces> workspaces = _ui->workspaces();
		if (!workspaces)
			return false;

		Ptr<Workspace> modelWorkspace = workspaces->itemById("FusionSolidEnvironment");
		if (!modelWorkspace) {
			std::string errorMessage;
			int errorCode = _app->getLastError(&errorMessage);
			if (GenericErrors::Ok != errorCode) {
				_ui->messageBox("failed find workspace item SolidScriptsAddinsPanel: " + errorMessage);
				return false;
			}
		}

		Ptr<ToolbarPanels> toolbarPanels = modelWorkspace->toolbarPanels();
		if (!toolbarPanels)
			return false;
		Ptr<ToolbarPanel> hypoDesignPanel = toolbarPanels->itemById("adskHypoDesignPanel");
		if (hypoDesignPanel)
			hypoDesignPanel->deleteMe();
		hypoDesignPanel = toolbarPanels->add("adskHypoDesignPanel", "Hypo gears");
		if (!hypoDesignPanel)
			return false;

		Ptr<ToolbarControls> toolbarcontrols = hypoDesignPanel->controls();
		if (!toolbarcontrols)
			return false;

		Ptr<CommandControl> hypoCtrl = toolbarcontrols->addCommand(hypoDesignCmdDef);
		if (!hypoCtrl) {
			std::string errorMessage;
			int errorCode = _app->getLastError(&errorMessage);
			if (GenericErrors::Ok != errorCode)
				_ui->messageBox(errorMessage);
			return false;
		}


		
		hypoCtrl->isPromoted(true);
		hypoCtrl->isPromotedByDefault(true);
		
		// now register the created event handler
		hypoDesignCmdDef->commandCreated()->add(&_hypoCmdCreated);

		return true;
	}

extern "C" XI_EXPORT bool stop(const char* context)
	{
	// check https://youtu.be/eL_uh8jZ-2Q?t=350
	// we must delete everything that's valid. This is extremely important for debugging
		if (_ui)
		{
			_ui->messageBox("Goodbye user");
			

			Ptr<Workspaces> workspaces = _ui->workspaces();
			if (!workspaces)
				return false;

			Ptr<Workspace> modelWorkspace = workspaces->itemById("FusionSolidEnvironment");
			if (!modelWorkspace)
				return false;

			Ptr<ToolbarPanels> toolbarPanels = modelWorkspace->toolbarPanels();
			if (!toolbarPanels)
				return false;

			Ptr<ToolbarPanel> furnitureDesignPanel = toolbarPanels->itemById("adskHypoDesignPanel");
			if (furnitureDesignPanel)
			{
				Ptr<ToolbarControls> furnitureDesignCtrls = furnitureDesignPanel->controls();
				if (!furnitureDesignCtrls)
					return false;

				Ptr<CommandControl> hypoCtrl = furnitureDesignCtrls->itemById("adskHypoDesignCmdDef");
				if (hypoCtrl)
					hypoCtrl->deleteMe();

				furnitureDesignPanel->deleteMe();
			}
			_ui = nullptr;
		}

		return true;
	}
}

#ifdef XI_WIN

#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hmodule, DWORD reason, LPVOID reserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif // XI_WIN



