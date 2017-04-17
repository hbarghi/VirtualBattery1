/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008,2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Kirill Andreev <andreev@iitp.ru>
 */

#ifndef HWMP_PROTOCOL_H
#define HWMP_PROTOCOL_H

#include "ns3/mesh-l2-routing-protocol.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/traced-value.h"
#include <vector>
#include <map>
#include <float.h>
#include <algorithm>

#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_integration.h>

#define CBR_ROUTE_EXPIRE_SECONDS 3
#define SOURCE_CBR_ROUTE_EXPIRE_MILLISECONDS 100

namespace ns3 {
class MeshPointDevice;
class Packet;
class Mac48Address;
class UniformRandomVariable;
namespace dot11s {
class HwmpProtocolMac;
class HwmpRtable;
class IePerr;
class IePreq;
class IePrep;
/**
 * \ingroup dot11s
 *
 * \brief Hybrid wireless mesh protocol -- a routing protocol of IEEE 802.11s draft.
 */


/* x1, abscissae common to the 10-, 21-, 43- and 87-point rule */
static const double x1[5] = {
  0.973906528517171720077964012084452,
  0.865063366688984510732096688423493,
  0.679409568299024406234327365114874,
  0.433395394129247190799265943165784,
  0.148874338981631210884826001129720
} ;

/* w10, weights of the 10-point formula */
static const double w10[5] = {
  0.066671344308688137593568809893332,
  0.149451349150580593145776339657697,
  0.219086362515982043995534934228163,
  0.269266719309996355091226921569469,
  0.295524224714752870173892994651338
} ;

/* x2, abscissae common to the 21-, 43- and 87-point rule */
static const double x2[5] = {
  0.995657163025808080735527280689003,
  0.930157491355708226001207180059508,
  0.780817726586416897063717578345042,
  0.562757134668604683339000099272694,
  0.294392862701460198131126603103866
} ;

/* w21a, weights of the 21-point formula for abscissae x1 */
static const double w21a[5] = {
  0.032558162307964727478818972459390,
  0.075039674810919952767043140916190,
  0.109387158802297641899210590325805,
  0.134709217311473325928054001771707,
  0.147739104901338491374841515972068
} ;

/* w21b, weights of the 21-point formula for abscissae x2 */
static const double w21b[6] = {
  0.011694638867371874278064396062192,
  0.054755896574351996031381300244580,
  0.093125454583697605535065465083366,
  0.123491976262065851077958109831074,
  0.142775938577060080797094273138717,
  0.149445554002916905664936468389821
} ;

/* x3, abscissae common to the 43- and 87-point rule */
static const double x3[11] = {
  0.999333360901932081394099323919911,
  0.987433402908088869795961478381209,
  0.954807934814266299257919200290473,
  0.900148695748328293625099494069092,
  0.825198314983114150847066732588520,
  0.732148388989304982612354848755461,
  0.622847970537725238641159120344323,
  0.499479574071056499952214885499755,
  0.364901661346580768043989548502644,
  0.222254919776601296498260928066212,
  0.074650617461383322043914435796506
} ;

/* w43a, weights of the 43-point formula for abscissae x1, x3 */
static const double w43a[10] = {
  0.016296734289666564924281974617663,
  0.037522876120869501461613795898115,
  0.054694902058255442147212685465005,
  0.067355414609478086075553166302174,
  0.073870199632393953432140695251367,
  0.005768556059769796184184327908655,
  0.027371890593248842081276069289151,
  0.046560826910428830743339154433824,
  0.061744995201442564496240336030883,
  0.071387267268693397768559114425516
} ;

 /* w43b, weights of the 43-point formula for abscissae x3 */
 static const double w43b[12] = {
   0.001844477640212414100389106552965,
   0.010798689585891651740465406741293,
   0.021895363867795428102523123075149,
   0.032597463975345689443882222526137,
   0.042163137935191811847627924327955,
   0.050741939600184577780189020092084,
   0.058379395542619248375475369330206,
   0.064746404951445885544689259517511,
   0.069566197912356484528633315038405,
   0.072824441471833208150939535192842,
   0.074507751014175118273571813842889,
   0.074722147517403005594425168280423
 } ;

 /* x4, abscissae of the 87-point rule */
 static const double x4[22] = {
   0.999902977262729234490529830591582,
   0.997989895986678745427496322365960,
   0.992175497860687222808523352251425,
   0.981358163572712773571916941623894,
   0.965057623858384619128284110607926,
   0.943167613133670596816416634507426,
   0.915806414685507209591826430720050,
   0.883221657771316501372117548744163,
   0.845710748462415666605902011504855,
   0.803557658035230982788739474980964,
   0.757005730685495558328942793432020,
   0.706273209787321819824094274740840,
   0.651589466501177922534422205016736,
   0.593223374057961088875273770349144,
   0.531493605970831932285268948562671,
   0.466763623042022844871966781659270,
   0.399424847859218804732101665817923,
   0.329874877106188288265053371824597,
   0.258503559202161551802280975429025,
   0.185695396568346652015917141167606,
   0.111842213179907468172398359241362,
   0.037352123394619870814998165437704
 } ;

 /* w87a, weights of the 87-point formula for abscissae x1, x2, x3 */
 static const double w87a[21] = {
   0.008148377384149172900002878448190,
   0.018761438201562822243935059003794,
   0.027347451050052286161582829741283,
   0.033677707311637930046581056957588,
   0.036935099820427907614589586742499,
   0.002884872430211530501334156248695,
   0.013685946022712701888950035273128,
   0.023280413502888311123409291030404,
   0.030872497611713358675466394126442,
   0.035693633639418770719351355457044,
   0.000915283345202241360843392549948,
   0.005399280219300471367738743391053,
   0.010947679601118931134327826856808,
   0.016298731696787335262665703223280,
   0.021081568889203835112433060188190,
   0.025370969769253827243467999831710,
   0.029189697756475752501446154084920,
   0.032373202467202789685788194889595,
   0.034783098950365142750781997949596,
   0.036412220731351787562801163687577,
   0.037253875503047708539592001191226
 } ;

 /* w87b, weights of the 87-point formula for abscissae x4    */
 static const double w87b[23] = {
   0.000274145563762072350016527092881,
   0.001807124155057942948341311753254,
   0.004096869282759164864458070683480,
   0.006758290051847378699816577897424,
   0.009549957672201646536053581325377,
   0.012329447652244853694626639963780,
   0.015010447346388952376697286041943,
   0.017548967986243191099665352925900,
   0.019938037786440888202278192730714,
   0.022194935961012286796332102959499,
   0.024339147126000805470360647041454,
   0.026374505414839207241503786552615,
   0.028286910788771200659968002987960,
   0.030052581128092695322521110347341,
   0.031646751371439929404586051078883,
   0.033050413419978503290785944862689,
   0.034255099704226061787082821046821,
   0.035262412660156681033782717998428,
   0.036076989622888701185500318003895,
   0.036698604498456094498018047441094,
   0.037120549269832576114119958413599,
   0.037334228751935040321235449094698,
   0.037361073762679023410321241766599
 } ;


static const double doubleExponentialAbcissas[] =
{
    // 1st layer abcissas: transformed 0, 1, 2, 3
    0.00000000000000000000,
    0.95136796407274694573,
    0.99997747719246159286,
    0.99999999999995705839,
    // 2nd layer abcissas: transformed 1/2, 3/2, 5/2
    0.67427149224843582608,
    0.99751485645722438683,
    0.99999998887566488198,
    // 3rd layer abcissas: transformed 1/4, 3/4, ...
    0.37720973816403417379,
    0.85956905868989663517,
    0.98704056050737689169,
    0.99968826402835320905,
    0.99999920473711471266,
    0.99999999995285644818,
    // 4th layer abcissas: transformed 1/8, 3/8, ...
    0.19435700332493543161,
    0.53914670538796776905,
    0.78060743898320029925,
    0.91487926326457461091,
    0.97396686819567744856,
    0.99405550663140214329,
    0.99906519645578584642,
    0.99990938469514399984,
    0.99999531604122052843,
    0.99999989278161241838,
    0.99999999914270509218,
    0.99999999999823216531,
    // 5th layer abcissa: transformed 1/16, 3/16, ...
    0.097923885287832333262,
    0.28787993274271591456,
    0.46125354393958570440,
    0.61027365750063894488,
    0.73101803479256151149,
    0.82331700550640237006,
    0.88989140278426019808,
    0.93516085752198468323,
    0.96411216422354729193,
    0.98145482667733517003,
    0.99112699244169880223,
    0.99610866543750854254,
    0.99845420876769773751,
    0.99945143443527460584,
    0.99982882207287494166,
    0.99995387100562796075,
    0.99998948201481850361,
    0.99999801714059543208,
    0.99999969889415261122,
    0.99999996423908091534,
    0.99999999678719909830,
    0.99999999978973286224,
    0.99999999999039393352,
    0.99999999999970809734,
    // 6th layer abcissas: transformed 1/32, 3/32, ...
    0.049055967305077886315,
    0.14641798429058794053,
    0.24156631953888365838,
    0.33314226457763809244,
    0.41995211127844715849,
    0.50101338937930910152,
    0.57558449063515165995,
    0.64317675898520470128,
    0.70355000514714201566,
    0.75669390863372994941,
    0.80279874134324126576,
    0.84221924635075686382,
    0.87543539763040867837,
    0.90301328151357387064,
    0.92556863406861266645,
    0.94373478605275715685,
    0.95813602271021369012,
    0.96936673289691733517,
    0.97797623518666497298,
    0.98445883116743083087,
    0.98924843109013389601,
    0.99271699719682728538,
    0.99517602615532735426,
    0.99688031812819187372,
    0.99803333631543375402,
    0.99879353429880589929,
    0.99928111192179195541,
    0.99958475035151758732,
    0.99976797159956083506,
    0.99987486504878034648,
    0.99993501992508242369,
    0.99996759306794345976,
    0.99998451990227082442,
    0.99999293787666288565,
    0.99999693244919035751,
    0.99999873547186590954,
    0.99999950700571943689,
    0.99999981889371276701,
    0.99999993755407837378,
    0.99999997987450320175,
    0.99999999396413420165,
    0.99999999832336194826,
    0.99999999957078777261,
    0.99999999989927772326,
    0.99999999997845533741,
    0.99999999999582460688,
    0.99999999999927152627,
    0.99999999999988636130,
    // 7th layer abcissas: transformed 1/64, 3/64, ...
    0.024539763574649160379,
    0.073525122985671294475,
    0.12222912220155764235,
    0.17046797238201051811,
    0.21806347346971200463,
    0.26484507658344795046,
    0.31065178055284596083,
    0.35533382516507453330,
    0.39875415046723775644,
    0.44078959903390086627,
    0.48133184611690504422,
    0.52028805069123015958,
    0.55758122826077823080,
    0.59315035359195315880,
    0.62695020805104287950,
    0.65895099174335012438,
    0.68913772506166767176,
    0.71750946748732412721,
    0.74407838354734739913,
    0.76886868676824658459,
    0.79191549237614211447,
    0.81326360850297385168,
    0.83296629391941087564,
    0.85108400798784873261,
    0.86768317577564598669,
    0.88283498824466895513,
    0.89661425428007602579,
    0.90909831816302043511,
    0.92036605303195280235,
    0.93049693799715340631,
    0.93957022393327475539,
    0.94766419061515309734,
    0.95485549580502268541,
    0.96121861515111640753,
    0.96682537031235585284,
    0.97174454156548730892,
    0.97604156025657673933,
    0.97977827580061576265,
    0.98301279148110110558,
    0.98579936302528343597,
    0.98818835380074264243,
    0.99022624046752774694,
    0.99195566300267761562,
    0.99341551316926403900,
    0.99464105571251119672,
    0.99566407681695316965,
    0.99651305464025377317,
    0.99721334704346870224,
    0.99778739195890653083,
    0.99825491617199629344,
    0.99863314864067747762,
    0.99893703483351217373,
    0.99917944893488591716,
    0.99937140114093768690,
    0.99952223765121720422,
    0.99963983134560036519,
    0.99973076151980848263,
    0.99980048143113838630,
    0.99985347277311141171,
    0.99989338654759256426,
    0.99992317012928932869,
    0.99994518061445869309,
    0.99996128480785666613,
    0.99997294642523223656,
    0.99998130127012072679,
    0.99998722128200062811,
    0.99999136844834487344,
    0.99999423962761663478,
    0.99999620334716617675,
    0.99999752962380516793,
    0.99999841381096473542,
    0.99999899541068996962,
    0.99999937270733536947,
    0.99999961398855024275,
    0.99999976602333243312,
    0.99999986037121459941,
    0.99999991800479471056,
    0.99999995264266446185,
    0.99999997311323594362,
    0.99999998500307631173,
    0.99999999178645609907,
    0.99999999558563361584,
    0.99999999767323673790,
    0.99999999879798350040,
    0.99999999939177687583,
    0.99999999969875436925,
    0.99999999985405611550,
    0.99999999993088839501,
    0.99999999996803321674,
    0.99999999998556879008,
    0.99999999999364632387,
    0.99999999999727404948,
    0.99999999999886126543,
    0.99999999999953722654,
    0.99999999999981720098,
    0.99999999999992987953
}; // end abcissas

static const double doubleExponentialWeights[] =
{
    // First layer weights
    1.5707963267948966192,
    0.230022394514788685,
    0.00026620051375271690866,
    1.3581784274539090834e-12,
    // 2nd layer weights
    0.96597657941230114801,
    0.018343166989927842087,
    2.1431204556943039358e-7,
    // 3rd layer weights
    1.3896147592472563229,
    0.53107827542805397476,
    0.076385743570832304188,
    0.0029025177479013135936,
    0.000011983701363170720047,
    1.1631165814255782766e-9,
    // 4th layer weights
    1.5232837186347052132,
    1.1934630258491569639,
    0.73743784836154784136,
    0.36046141846934367417,
    0.13742210773316772341,
    0.039175005493600779072,
    0.0077426010260642407123,
    0.00094994680428346871691,
    0.000062482559240744082891,
    1.8263320593710659699e-6,
    1.8687282268736410132e-8,
    4.9378538776631926964e-11,
    //  5th layer weights
    1.5587733555333301451,
    1.466014426716965781,
    1.297475750424977998,
    1.0816349854900704074,
    0.85017285645662006895,
    0.63040513516474369106,
    0.44083323627385823707,
    0.290240679312454185,
    0.17932441211072829296,
    0.10343215422333290062,
    0.055289683742240583845,
    0.027133510013712003219,
    0.012083543599157953493,
    0.0048162981439284630173,
    0.0016908739981426396472,
    0.00051339382406790336017,
    0.00013205234125609974879,
    0.000028110164327940134749,
    4.8237182032615502124e-6,
    6.4777566035929719908e-7,
    6.5835185127183396672e-8,
    4.8760060974240625869e-9,
    2.5216347918530148572e-10,
    8.6759314149796046502e-12,
    // 6th layer weights
    1.5677814313072218572,
    1.5438811161769592204,
    1.4972262225410362896,
    1.4300083548722996676,
    1.3452788847662516615,
    1.2467012074518577048,
    1.1382722433763053734,
    1.0240449331118114483,
    0.90787937915489531693,
    0.79324270082051671787,
    0.68306851634426375464,
    0.57967810308778764708,
    0.48475809121475539287,
    0.39938474152571713515,
    0.32408253961152890402,
    0.258904639514053516,
    0.20352399885860174519,
    0.15732620348436615027,
    0.11949741128869592428,
    0.089103139240941462841,
    0.065155533432536205042,
    0.046668208054846613644,
    0.032698732726609031113,
    0.022379471063648476483,
    0.014937835096050129696,
    0.0097072237393916892692,
    0.0061300376320830301252,
    0.0037542509774318343023,
    0.0022250827064786427022,
    0.0012733279447082382027,
    0.0007018595156842422708,
    0.00037166693621677760301,
    0.00018856442976700318572,
    0.000091390817490710122732,
    0.000042183183841757600604,
    0.000018481813599879217116,
    7.6595758525203162562e-6,
    2.9916615878138787094e-6,
    1.0968835125901264732e-6,
    3.7595411862360630091e-7,
    1.1992442782902770219e-7,
    3.5434777171421953043e-8,
    9.6498888961089633609e-9,
    2.4091773256475940779e-9,
    5.482835779709497755e-10,
    1.1306055347494680536e-10,
    2.0989335404511469109e-11,
    3.4841937670261059685e-12,
    // 7th layer weights
    1.5700420292795931467,
    1.5640214037732320999,
    1.5520531698454121192,
    1.5342817381543034316,
    1.5109197230741697127,
    1.48224329788553807,
    1.4485862549613225916,
    1.4103329714462590129,
    1.3679105116808964881,
    1.3217801174437728579,
    1.2724283455378627082,
    1.2203581095793582207,
    1.1660798699324345766,
    1.1101031939653403796,
    1.0529288799552666556,
    0.99504180404613271514,
    0.93690461274566793366,
    0.87895234555278212039,
    0.82158803526696470334,
    0.7651792989089561367,
    0.71005590120546898385,
    0.65650824613162753076,
    0.60478673057840362158,
    0.55510187800363350959,
    0.5076251588319080997,
    0.4624903980553677613,
    0.41979566844501548066,
    0.37960556938665160999,
    0.3419537959230168323,
    0.30684590941791694932,
    0.27426222968906810637,
    0.24416077786983990868,
    0.21648020911729617038,
    0.19114268413342749532,
    0.16805663794826916233,
    0.14711941325785693248,
    0.12821973363120098675,
    0.11123999898874453035,
    0.096058391865189467849,
    0.082550788110701737654,
    0.070592469906866999352,
    0.060059642358636300319,
    0.05083075757257047107,
    0.042787652157725676034,
    0.035816505604196436523,
    0.029808628117310126969,
    0.024661087314753282511,
    0.020277183817500123926,
    0.016566786254247575375,
    0.013446536605285730674,
    0.010839937168255907211,
    0.0086773307495391815854,
    0.0068957859690660035329,
    0.0054388997976239984331,
    0.0042565295990178580165,
    0.0033044669940348302363,
    0.0025440657675291729678,
    0.0019418357759843675814,
    0.0014690143599429791058,
    0.0011011261134519383862,
    0.00081754101332469493115,
    0.00060103987991147422573,
    0.00043739495615911687786,
    0.00031497209186021200274,
    0.00022435965205008549104,
    0.00015802788400701191949,
    0.00011002112846666697224,
    0.000075683996586201477788,
    0.000051421497447658802092,
    0.0000344921247593431977,
    0.000022832118109036146591,
    0.000014908514031870608449,
    9.5981941283784710776e-6,
    6.0899100320949039256e-6,
    3.8061983264644899045e-6,
    2.3421667208528096843e-6,
    1.4183067155493917523e-6,
    8.4473756384859863469e-7,
    4.9458288702754198508e-7,
    2.8449923659159806339e-7,
    1.6069394579076224911e-7,
    8.9071395140242387124e-8,
    4.8420950198072369669e-8,
    2.579956822953589238e-8,
    1.3464645522302038796e-8,
    6.8784610955899001111e-9,
    3.4371856744650090511e-9,
    1.6788897682161906807e-9,
    8.0099784479729665356e-10,
    3.7299501843052790038e-10,
    1.6939457789411646876e-10,
    7.4967397573818224522e-11,
    3.230446433325236576e-11,
    1.3542512912336274432e-11,
    5.5182369468174885821e-12,
    2.1835922099233609052e-12
}; // end weights

/*! Numerical integration in one dimension using the double expontial method of M. Mori. */
template<class TFunctionObject>
class DEIntegrator
{
public:
    /*! Integrate an analytic function over a finite interval. @return The value of the integral. */
    static double Integrate
    (
        const TFunctionObject& f,       //!< [in] integrand
        double a,                       //!< [in] left limit of integration
        double b,                       //!< [in] right limit of integration
        double alpha,
        double beta,
        double x0,
        double targetAbsoluteError,     //!< [in] desired bound on error
        int& numFunctionEvaluations,    //!< [out] number of function evaluations used
        double& errorEstimate           //!< [out] estimated error in integration
    )
    {
        // Apply the linear change of variables x = ct + d
        // $$\int_a^b f(x) dx = c \int_{-1}^1 f( ct + d ) dt$$
        // c = (b-a)/2, d = (a+b)/2

        double c = 0.5*(b - a);
        double d = 0.5*(a + b);

        return IntegrateCore
        (
            f,
            c,
            d,
            alpha,
            beta,
            x0,
            targetAbsoluteError,
            numFunctionEvaluations,
            errorEstimate,
            doubleExponentialAbcissas,
            doubleExponentialWeights
        );
    }

    /*! Integrate an analytic function over a finite interval.
        This version overloaded to not require arguments passed in for
        function evaluation counts or error estimates.
        @return The value of the integral.
    */
    static double Integrate
    (
        const TFunctionObject& f,       //!< [in] integrand
        double a,                       //!< [in] left limit of integration
        double b,                       //!< [in] right limit of integration
        double alpha,
        double beta,
        double x0,
        double targetAbsoluteError      //!< [in] desired bound on error
    )
    {
        int numFunctionEvaluations;
        double errorEstimate;
        return Integrate
        (
            f,
            a,
            b,
            alpha,
            beta,
            x0,
            targetAbsoluteError,
            numFunctionEvaluations,
            errorEstimate
        );
    }



private:
    // Integrate f(cx + d) with the given integration constants
    static double IntegrateCore
    (
        const TFunctionObject& f,
        double c,   // slope of change of variables
        double d,   // intercept of change of variables
        double alpha,
        double beta,
        double x0,
        double targetAbsoluteError,
        int& numFunctionEvaluations,
        double& errorEstimate,
        const double* abcissas,
        const double* weights
    )
    {
        targetAbsoluteError /= c;

        // Offsets to where each level's integration constants start.
        // The last element is not a beginning but an end.
        int offsets[] = {1, 4, 7, 13, 25, 49, 97, 193};
        int numLevels = sizeof(offsets)/sizeof(int) - 1;

        double newContribution = 0.0;
        double integral = 0.0;
        errorEstimate = DBL_MAX;
        double h = 1.0;
        double previousDelta, currentDelta = DBL_MAX;

        integral = f(c*abcissas[0] + d,alpha,beta,x0) * weights[0];
        int i;
        for (i = offsets[0]; i != offsets[1]; ++i)
            integral += weights[i]*(f(c*abcissas[i] + d,alpha,beta,x0) + f(-c*abcissas[i] + d,alpha,beta,x0));

        for (int level = 1; level != numLevels; ++level)
        {
            h *= 0.5;
            newContribution = 0.0;
            for (i = offsets[level]; i != offsets[level+1]; ++i)
                newContribution += weights[i]*(f(c*abcissas[i] + d,alpha,beta,x0) + f(-c*abcissas[i] + d,alpha,beta,x0));
            newContribution *= h;

            // difference in consecutive integral estimates
            previousDelta = currentDelta;
            currentDelta = fabs(0.5*integral - newContribution);
            integral = 0.5*integral + newContribution;

            // Once convergence kicks in, error is approximately squared at each step.
            // Determine whether we're in the convergent region by looking at the trend in the error.
            if (level == 1)
                continue; // previousDelta meaningless, so cannot check convergence.

            // Exact comparison with zero is harmless here.  Could possibly be replaced with
            // a small positive upper limit on the size of currentDelta, but determining
            // that upper limit would be difficult.  At worse, the loop is executed more
            // times than necessary.  But no infinite loop can result since there is
            // an upper bound on the loop variable.
            if (currentDelta == 0.0)
                break;
            double r = log( currentDelta )/log( previousDelta );  // previousDelta != 0 or would have been kicked out previously

            if (r > 1.9 && r < 2.1)
            {
                // If convergence theory applied perfectly, r would be 2 in the convergence region.
                // r close to 2 is good enough. We expect the difference between this integral estimate
                // and the next one to be roughly delta^2.
                errorEstimate = currentDelta*currentDelta;
            }
            else
            {
                // Not in the convergence region.  Assume only that error is decreasing.
                errorEstimate = currentDelta;
            }

            if (errorEstimate < 0.1*targetAbsoluteError)
                break;
        }

        numFunctionEvaluations = 2*i - 1;
        errorEstimate *= c;
        return c*integral;
    }

};


struct CbrConnection
{
  Mac48Address destination;
  Mac48Address source;
  uint8_t cnnType;
  Ipv4Address srcIpv4Addr;
  Ipv4Address dstIpv4Addr;
  uint16_t srcPort;
  uint16_t dstPort;
  Time whenExpires;
  Mac48Address prevMac;
  Mac48Address nextMac;
  inline bool operator == (const CbrConnection &o) const {
        return ( o.dstIpv4Addr == dstIpv4Addr && o.dstPort == dstPort && o.srcIpv4Addr == srcIpv4Addr && o.srcPort == srcPort );
                        //||    		   ( o.dstIpv4Addr == srcIpv4Addr && o.dstPort == srcPort && o.srcIpv4Addr == dstIpv4Addr && o.srcPort == dstPort );
  }
};

typedef std::vector<CbrConnection> CbrConnectionsVector;

class HwmpProtocol : public MeshL2RoutingProtocol
{
public:
  double  rescale_error (double err, const double result_abs, const double result_asc);
  int  mygsl_integration_qng (const gsl_function *f,
                       double a, double b,
                       double epsabs, double epsrel,
                       double * result, double * abserr, size_t * neval);

  static TypeId GetTypeId ();
  HwmpProtocol ();
  ~HwmpProtocol ();
  void DoDispose ();
  /**
   * \brief structure of unreachable destination - address and sequence number
   */
  typedef struct
  {
    Mac48Address destination;
    uint32_t seqnum;
  } FailedDestination;
  /// Route request, inherited from MeshL2RoutingProtocol
  bool RequestRoute (uint32_t  sourceIface, const Mac48Address source, const Mac48Address destination,
                     Ptr<const Packet>  packet, uint16_t  protocolType, RouteReplyCallback  routeReply);
  /// Cleanup packet from all tags
  bool RemoveRoutingStuff (uint32_t fromIface, const Mac48Address source,
                           const Mac48Address destination, Ptr<Packet>  packet, uint16_t&  protocolType);
  /**
   * \brief Install HWMP on given mesh point.
   *
   * Installing protocol cause installing its interface MAC plugins.
   *
   * Also MP aggregates all installed protocols, HWMP protocol can be accessed
   * via MeshPointDevice::GetObject<dot11s::HwmpProtocol>();
   */
  bool Install (Ptr<MeshPointDevice>);
  void PeerLinkStatus (Mac48Address meshPontAddress, Mac48Address peerAddress, uint32_t interface,bool status);
  ///\brief This callback is used to obtain active neighbours on a given interface
  ///\param cb is a callback, which returns a list of addresses on given interface (uint32_t)
  void SetNeighboursCallback (Callback<std::vector<Mac48Address>, uint32_t> cb);
  ///\name Proactive PREQ mechanism:
  ///\{
  void SetRoot ();
  void UnsetRoot ();
  ///\}
  ///\brief Statistics:
  void Report (std::ostream &) const;
  void ResetStats ();
  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

private:
  friend class HwmpProtocolMac;

  virtual void DoInitialize ();

  HwmpProtocol& operator= (const HwmpProtocol &);
  HwmpProtocol (const HwmpProtocol &);

  /**
   * \brief Structure of path error: IePerr and list of receivers:
   * interfaces and MAC address
   */
  struct PathError
  {
    std::vector<FailedDestination> destinations; ///< destination list: Mac48Address and sequence number
    std::vector<std::pair<uint32_t, Mac48Address> > receivers; ///< list of PathError receivers (in case of unicast PERR)
  };
  /// Packet waiting its routing information
  struct QueuedPacket
  {
    Ptr<Packet> pkt; ///< the packet
    Mac48Address src; ///< src address
    Mac48Address dst; ///< dst address
    uint16_t protocol; ///< protocol number
    uint8_t cnnType;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;
    uint32_t inInterface; ///< incoming device interface ID. (if packet has come from upper layers, this is Mesh point ID)
    RouteReplyCallback reply; ///< how to reply

    QueuedPacket ();
  };
  struct CnnBasedPreqEvent {
    EventId preqTimeout;
    Time whenScheduled;
    Mac48Address destination;
    Mac48Address source;
    uint8_t cnnType;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;
  };
  std::vector<CnnBasedPreqEvent> m_cnnBasedPreqTimeouts;
  typedef std::map<uint32_t, Ptr<HwmpProtocolMac> > HwmpProtocolMacMap;
  /// Like RequestRoute, but for unicast packets
  bool ForwardUnicast (uint32_t  sourceIface, const Mac48Address source, const Mac48Address destination,
                       Ptr<Packet>  packet, uint16_t  protocolType, RouteReplyCallback  routeReply, uint32_t ttl);

  ///\name Interaction with HWMP MAC plugin
  //\{
  void ReceivePreq (IePreq preq, Mac48Address from, uint32_t interface, Mac48Address fromMp, uint32_t metric);
  void ReceivePrep (IePrep prep, Mac48Address from, uint32_t interface, Mac48Address fromMp, uint32_t metric);
  void ReceivePerr (std::vector<FailedDestination>, Mac48Address from, uint32_t interface, Mac48Address fromMp);
  void SendPrep (
    Mac48Address src,
    Mac48Address dst,
    Mac48Address retransmitter,
    uint32_t initMetric,
    uint8_t cnnType,
    Ipv4Address srcIpv4Addr,
    Ipv4Address dstIpv4Addr,
    uint16_t srcPort,
    uint16_t dstPort,
    uint32_t originatorDsn,
    uint32_t destinationSN,
    uint32_t lifetime,
    uint32_t interface);
  void InsertCbrCnnAtSourceIntoSourceCbrCnnsVector(
                        Mac48Address destination,
                        Mac48Address source,
                        uint8_t cnnType,
                    Ipv4Address srcIpv4Addr,
                    Ipv4Address dstIpv4Addr,
                    uint16_t srcPort,
                    uint16_t dstPort,
                    Mac48Address prevHop,
                    Mac48Address nextHop
                  );
  void SourceCbrRouteExtend(
                        Mac48Address destination,
                        Mac48Address source,
                        uint8_t cnnType,
                    Ipv4Address srcIpv4Addr,
                    Ipv4Address dstIpv4Addr,
                    uint16_t srcPort,
                    uint16_t dstPort
                  );
  void InsertCbrCnnIntoCbrCnnsVector(
                    Mac48Address destination,
                    Mac48Address source,
                    uint8_t cnnType,
                    Ipv4Address srcIpv4Addr,
                    Ipv4Address dstIpv4Addr,
                    uint16_t srcPort,
                    uint16_t dstPort,
                    Mac48Address prevHop,
                    Mac48Address nextHop
                  );
  void CbrRouteExtend(
                    Mac48Address destination,
                    Mac48Address source,
                    uint8_t cnnType,
                    Ipv4Address srcIpv4Addr,
                    Ipv4Address dstIpv4Addr,
                    uint16_t srcPort,
                    uint16_t dstPort
                  );
  void CbrRouteExpire(CbrConnection cbrCnn);
  void CheckCbrRoutes4Expiration();
  /**
   * \brief forms a path error information element when list of destination fails on a given interface
   * \attention removes all entries from routing table!
   */
  PathError MakePathError (std::vector<FailedDestination> destinations);
  ///\brief Forwards a received path error
  void ForwardPathError (PathError perr);
  ///\brief Passes a self-generated PERR to interface-plugin
  void InitiatePathError (PathError perr);
  /// \return list of addresses where a PERR should be sent to
  std::vector<std::pair<uint32_t, Mac48Address> > GetPerrReceivers (std::vector<FailedDestination> failedDest);

  /// \return list of addresses where a PERR should be sent to
  std::vector<Mac48Address> GetPreqReceivers (uint32_t interface);
  /// \return list of addresses where a broadcast should be
  //retransmitted
  std::vector<Mac48Address> GetBroadcastReceivers (uint32_t interface);
  /**
   * \brief MAC-plugin asks whether the frame can be dropped. Protocol automatically updates seqno.
   *
   * \return true if frame can be dropped
   * \param seqno is the sequence number of source
   * \param source is the source address
   */
  bool DropDataFrame (uint32_t seqno, Mac48Address source);
  //\}
  /// Route discovery time:
  TracedCallback<Time> m_routeDiscoveryTimeCallback;
  TracedCallback<> m_txed4mSourceCallback;
  TracedCallback<> m_wannaTx4mSourceCallback;
  TracedCallback<Ipv4Address,Ipv4Address,uint16_t,uint16_t,bool> m_CbrCnnStateChanged;
  TracedCallback<Ptr<Packet> > m_packetBufferredAtSource;
  ///\name Methods related to Queue/Dequeue procedures
  ///\{
  bool QueuePacket (QueuedPacket packet);
  QueuedPacket  DequeueFirstPacketByCnnParams (
                        Mac48Address dst,
                        Mac48Address src,
                        uint8_t cnnType,
                        Ipv4Address srcIpv4Addr,
                        Ipv4Address dstIpv4Addr,
                        uint16_t srcPort,
                        uint16_t dstPort
  );
  QueuedPacket  DequeueFirstPacketByDst (Mac48Address dst);
  QueuedPacket  DequeueFirstPacket ();
  void ReactivePathResolved (Mac48Address dst);
  void CnnBasedReactivePathResolved (
      Mac48Address dst,
      Mac48Address src,
      uint8_t cnnType,
      Ipv4Address srcIpv4Addr,
      Ipv4Address dstIpv4Addr,
      uint16_t srcPort,
      uint16_t dstPort
                    );
  double IntegrateNumerical(double a,double b,double alpha,double beta,double x0,double threshold);
  void ProactivePathResolved ();
  ///\}
  ///\name Methods responsible for path discovery retry procedure:
  ///\{
  /**
   * \brief checks when the last path discovery procedure was started for a given destination.
   *
   * If the retry counter has not achieved the maximum level - preq should not be sent
   */
  bool  ShouldSendPreq (Mac48Address dst);
  bool  CnnBasedShouldSendPreq (
            Mac48Address dst,
            Mac48Address src,
            uint8_t cnnType,
            Ipv4Address srcIpv4Addr,
            Ipv4Address dstIpv4Addr,
            uint16_t srcPort,
            uint16_t dstPort
  );

  /**
   * \brief Generates PREQ retry when retry timeout has expired and route is still unresolved.
   *
   * When PREQ retry has achieved the maximum level - retry mechanism should be canceled
   */
  void  RetryPathDiscovery (Mac48Address dst, uint8_t numOfRetry);
  void  CnnBasedRetryPathDiscovery (
                CnnBasedPreqEvent preqEvent,
                uint8_t numOfRetry
  );
  /// Proactive Preq routines:
  void SendProactivePreq ();
  ///\}
  ///\return address of MeshPointDevice
  Mac48Address GetAddress ();
  ///\name Methods needed by HwmpMacLugin to access protocol parameters:
  ///\{
  bool GetDoFlag ();
  bool GetRfFlag ();
  Time GetPreqMinInterval ();
  Time GetPerrMinInterval ();
  uint8_t GetMaxTtl ();
  uint32_t GetNextPreqId ();
  uint32_t GetNextHwmpSeqno ();
  uint32_t GetActivePathLifetime ();
  uint8_t GetUnicastPerrThreshold ();
  ///\}
private:
  ///\name Statistics:
  ///\{
  struct Statistics
  {
    uint16_t txUnicast;
    uint16_t txBroadcast;
    uint32_t txBytes;
    uint16_t droppedTtl;
    uint16_t totalQueued;
    uint16_t totalDropped;
    uint16_t initiatedPreq;
    uint16_t initiatedPrep;
    uint16_t initiatedPerr;

    void Print (std::ostream & os) const;
    Statistics ();
  };
  Statistics m_stats;
  ///\}
  HwmpProtocolMacMap m_interfaces;
  Mac48Address m_address;
  uint32_t m_dataSeqno;
  uint32_t m_hwmpSeqno;
  uint32_t m_preqId;
  ///\name Sequence number filters
  ///\{
  /// Data sequence number database
  std::map<Mac48Address, uint32_t> m_lastDataSeqno;

  struct CnnBasedSeqnoMetricDprobDatabase {
    Mac48Address originatorAddress;
    uint32_t originatorSeqNumber;
    Mac48Address destinationAddress;
    uint32_t destinationSeqNumber;
    uint32_t metric;
    uint32_t dProb;
    uint8_t cnnType;
    Ipv4Address srcIpv4Addr;
    Ipv4Address dstIpv4Addr;
    uint16_t srcPort;
    uint16_t dstPort;
  };
  std::vector<CnnBasedSeqnoMetricDprobDatabase> m_hwmpSeqnoMetricDprobDatabase;

  /// keeps HWMP seqno (first in pair) and HWMP metric (second in pair) for each address
//  std::map<Mac48Address, std::pair<uint32_t, uint32_t> > m_hwmpSeqnoMetricDatabase;
//  std::map<Mac48Address, std::pair<uint32_t, std::pair<uint32_t, uint32_t> > > m_hwmpSeqnoMetricDatabase4prep;
  ///\}

  /// Routing table
  Ptr<HwmpRtable> m_rtable;

  ///\name Timers:
  //\{
  struct PreqEvent {
    EventId preqTimeout;
    Time whenScheduled;
  };
  std::map<Mac48Address, PreqEvent> m_preqTimeouts;
  EventId m_proactivePreqTimer;
  /// Random start in Proactive PREQ propagation
  Time m_randomStart;
  ///\}
  /// Packet Queue
  std::vector<QueuedPacket> m_rqueue;
  ///\name HWMP-protocol parameters (attributes of GetTypeId)
  ///\{
  uint16_t m_maxQueueSize;
  uint8_t m_dot11MeshHWMPmaxPREQretries;
  Time m_dot11MeshHWMPnetDiameterTraversalTime;
  Time m_dot11MeshHWMPpreqMinInterval;
  Time m_dot11MeshHWMPperrMinInterval;
  Time m_dot11MeshHWMPactiveRootTimeout;
  Time m_dot11MeshHWMPactivePathTimeout;
  Time m_dot11MeshHWMPpathToRootInterval;
  Time m_dot11MeshHWMPrannInterval;
  bool m_isRoot;
  uint8_t m_maxTtl;
  uint8_t m_unicastPerrThreshold;
  uint8_t m_unicastPreqThreshold;
  uint8_t m_unicastDataThreshold;
  bool m_doFlag;
  bool m_rfFlag;
  ///\}
  /// Random variable for random start time
  Ptr<UniformRandomVariable> m_coefficient;
  Callback <std::vector<Mac48Address>, uint32_t> m_neighboursCallback;

  CbrConnectionsVector m_cbrConnections;
  CbrConnectionsVector m_sourceCbrConnections;
  CbrConnectionsVector m_notRoutedCbrConnections;
};
} // namespace dot11s
} // namespace ns3
#endif
