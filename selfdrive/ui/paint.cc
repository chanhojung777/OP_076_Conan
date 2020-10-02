#include "ui.hpp"
#include <assert.h>
#include <map>
#include <cmath>
#include "common/util.h"
#include "common/params.h"
#include "common/swaglog.h"

#define NANOVG_GLES3_IMPLEMENTATION

#include "nanovg_gl.h"
#include "nanovg_gl_utils.h"

extern "C"{
#include "common/glutil.h"
}

// TODO: this is also hardcoded in common/transformations/camera.py
const mat3 intrinsic_matrix = (mat3){{
  910., 0., 582.,
  0., 910., 437.,
  0.,   0.,   1.
}};

const uint8_t alert_colors[][4] = {
  [STATUS_STOPPED] = {0x07, 0x23, 0x39, 0xe1},
  [STATUS_DISENGAGED] = {0x17, 0x33, 0x49, 0xc8},
  [STATUS_ENGAGED] = {0x17, 0x86, 0x44, 0x51},
  [STATUS_WARNING] = {0xDA, 0x6F, 0x25, 0x51},
  [STATUS_ALERT] = {0xC9, 0x22, 0x31, 0xe1},
};

float  fFontSize = 0.8;


void ui_awake_aleat(UIState *s, bool awake)
{
   s->is_awake_command = awake;
}

static void ui_print(UIState *s, int x, int y,  const char* fmt, ... )
{
  //char speed_str[512];  
  char* msg_buf = NULL;
  va_list args;
  va_start(args, fmt);
  vasprintf( &msg_buf, fmt, args);
  va_end(args);

  nvgText(s->vg, x, y, msg_buf, NULL);
}


// Projects a point in car to space to the corresponding point in full frame
// image space.
vec3 car_space_to_full_frame(const UIState *s, vec4 car_space_projective) 
{
  const UIScene *scene = &s->scene;

  // We'll call the car space point p.
  // First project into normalized image coordinates with the extrinsics matrix.
  const vec4 Ep4 = matvecmul(scene->extrinsic_matrix, car_space_projective);

  // The last entry is zero because of how we store E (to use matvecmul).
  const vec3 Ep = {{Ep4.v[0], Ep4.v[1], Ep4.v[2]}};
  const vec3 KEp = matvecmul3(intrinsic_matrix, Ep);

  // Project.
  const vec3 p_image = {{KEp.v[0] / KEp.v[2], KEp.v[1] / KEp.v[2], 1.}};
  return p_image;
}

// Calculate an interpolation between two numbers at a specific increment
static float lerp(float v0, float v1, float t) 
{
  return (1 - t) * v0 + t * v1;
}

static void ui_draw_text(NVGcontext *vg, float x, float y, const char* string, float size, NVGcolor color, int font)
{
  nvgFontFaceId(vg, font);
  nvgFontSize(vg, size*fFontSize);
  nvgFillColor(vg, color);
  nvgText(vg, x, y, string, NULL);
}

static void draw_chevron(UIState *s, float x_in, float y_in, float sz,
                          NVGcolor fillColor, NVGcolor glowColor) {
  const vec4 p_car_space = (vec4){{x_in, y_in, 0., 1.}};
  const vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);

  float x = p_full_frame.v[0];
  float y = p_full_frame.v[1];
  if (x < 0 || y < 0.){
    return;
  }

  sz *= 30;
  sz /= (x_in / 3 + 30);
  if (sz > 30) sz = 30;
  if (sz < 15) sz = 15;

  // glow
  float g_xo = sz/5;
  float g_yo = sz/10;
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
  nvgLineTo(s->vg, x, y-g_xo);
  nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
  nvgLineTo(s->vg, x, y);
  nvgLineTo(s->vg, x-(sz*1.25), y+sz);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);
}


// LOGW("got CarVin %s", value_vin);
// 
static void ui_draw_circle_image(NVGcontext *vg, float x, float y, int size, int image, NVGcolor color, float img_alpha, float angleSteers = 0) 
{
  const int img_size = size * 1.5;
  float img_rotation =  angleSteers/180*3.141592;

  int ct_pos = -size * 0.75;

  nvgBeginPath(vg);
  nvgCircle(vg, x, y + (bdr_s * 1.5), size);
  nvgFillColor(vg, color);
  nvgFill(vg);

  nvgSave( vg );
  nvgTranslate(vg,x,(y + (bdr_s*1.5)));
  nvgRotate(vg,-img_rotation);


  //LOGW("angleSteers = %.1f", angleSteers );
  //LOGW("image %d,%d  %.1f,%.1f", img_x, img_y, x, (y + (bdr_s*1.5)) );
  ui_draw_image(vg, ct_pos, ct_pos, img_size, img_size, image, img_alpha);

  nvgRestore(vg);
}

static void ui_draw_circle_image(NVGcontext *vg, float x, float y, int size, int image, bool active) 
{
  float bg_alpha = active ? 0.3f : 0.1f;
  float img_alpha = active ? 1.0f : 0.15f;
  ui_draw_circle_image(vg, x, y, size, image, nvgRGBA(0, 0, 0, (255 * bg_alpha)), img_alpha);
}

static void draw_lead(UIState *s, float d_rel, float v_rel, float y_rel)
{
  // Draw lead car indicator
  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }
  draw_chevron(s, d_rel+2.7, y_rel, 25, nvgRGBA(201, 34, 49, fillAlpha), COLOR_YELLOW);
}

// static void ui_draw_lane_line(UIState *s, const model_path_vertices_data *pvd, NVGcolor color) 
// {
//   if (pvd->cnt == 0) return;

//   nvgBeginPath(s->vg);
//   nvgMoveTo(s->vg, pvd->v[0].x, pvd->v[0].y);
//   for (int i=1; i<pvd->cnt; i++) {
//     nvgLineTo(s->vg, pvd->v[i].x, pvd->v[i].y);
//   }
//   nvgClosePath(s->vg);
//   nvgFillColor(s->vg, color);//blue, , nvgRGBA( 0, 120, 0, 150)); //< green 
//   nvgFill(s->vg);

//   // nvgBeginPath(s->vg);
//   // bool started = false;
//   // for (int i=0; i<pvd->cnt; i++) {
//   //   float x = pvd->v[i].x;
//   //   float y = pvd->v[i].y;
//   //   if (x < 0 || y < 0.) {
//   //     continue;
//   //   }
//   //   if (!started) {
//   //     nvgMoveTo(s->vg, x, y);
//   //     started = true;
//   //   } else {
//   //     nvgLineTo(s->vg, x, y);
//   //   }
//   // }
//   // nvgClosePath(s->vg);
//   // nvgFillColor(s->vg, nvgRGBA(0, 0, 200, 200));//blue, , nvgRGBA( 0, 120, 0, 150)); //< green 
//   // nvgFill(s->vg);
// }

static void ui_draw_lane_line(UIState *s, const model_path_vertices_data *pvd, NVGcolor color) 
{
  nvgBeginPath(s->vg);
  bool started = false;
  for (int i=0; i<pvd->cnt; i++) {
    float x = pvd->v[i].x;
    float y = pvd->v[i].y;
    if (x < 0 || y < 0.) {
      continue;
    }
    if (!started) {
      nvgMoveTo(s->vg, x, y);
      started = true;
    } else {
      nvgLineTo(s->vg, x, y);
    }
  }
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
}

static void update_track_data(UIState *s, bool is_mpc, track_vertices_data *pvd) 
{
  const UIScene *scene = &s->scene;
  const PathData path = scene->model.path;
  const float *mpc_x_coords = &scene->mpc_x[0];
  const float *mpc_y_coords = &scene->mpc_y[0];

  bool started = false;
  float off = is_mpc?0.3:0.5;
  float lead_d = scene->lead_d_rel*2.;
  float path_height = is_mpc?(lead_d>5.)?fmin(lead_d, 25.)-fmin(lead_d*0.35, 10.):20.
                            :(lead_d>0.)?fmin(lead_d, 50.)-fmin(lead_d*0.35, 10.):49.;
  if( !(s->livempc_or_radarstate_changed & 0x02) )
    is_mpc = 0;
  
  pvd->cnt = 0;
  // left side up
  for (int i=0; i<=path_height; i++) {
    float px, py, mpx;
    if (is_mpc) {
      mpx = i==0?0.0:mpc_x_coords[i];
      px = lerp(mpx+1.0, mpx, i/100.0);
      py = mpc_y_coords[i] - off;
    } else {
      px = lerp(i+1.0, i, i/100.0);
      py = path.points[i] - off;
    }

    vec4 p_car_space = (vec4){{px, py, 0., 1.}};
    vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);
    if (p_full_frame.v[0] < 0. || p_full_frame.v[1] < 0.) {
      continue;
    }
    pvd->v[pvd->cnt].x = p_full_frame.v[0];
    pvd->v[pvd->cnt].y = p_full_frame.v[1];
    pvd->cnt += 1;
  }

  // right side down
  for (int i=path_height; i>=0; i--) {
    float px, py, mpx;
    if (is_mpc) {
      mpx = i==0?0.0:mpc_x_coords[i];
      px = lerp(mpx+1.0, mpx, i/100.0);
      py = mpc_y_coords[i] + off;
    } else {
      px = lerp(i+1.0, i, i/100.0);
      py = path.points[i] + off;
    }

    vec4 p_car_space = (vec4){{px, py, 0., 1.}};
    vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);
    pvd->v[pvd->cnt].x = p_full_frame.v[0];
    pvd->v[pvd->cnt].y = p_full_frame.v[1];
    pvd->cnt += 1;
  }
}

static void update_all_track_data(UIState *s) 
{
  const UIScene *scene = &s->scene;
  // Draw vision path
  update_track_data(s, false, &s->track_vertices[0]);
  if (scene->engaged) {
    // Draw MPC path when engaged
    update_track_data(s, true, &s->track_vertices[1]);
  }
}



static void ui_draw_track(UIState *s, bool is_mpc, track_vertices_data *pvd) 
{
 if (pvd->cnt == 0) return;

  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, pvd->v[0].x, pvd->v[0].y);
  for (int i=1; i<pvd->cnt; i++) {
    nvgLineTo(s->vg, pvd->v[i].x, pvd->v[i].y);
  }
  nvgClosePath(s->vg);

  NVGpaint track_bg;
  int red_lvl = 0;
  int green_lvl = 0;
  int blue_lvl = 0;
  if (is_mpc) {
    // Draw colored MPC track Kegman's
    if (s->scene.kegman.steerOverride) {
      track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
        COLOR_WHITE, COLOR_WHITE_ALPHA(0));
    } else {
      int torque_scale = (int)fabs(255*(float)s->scene.kegman.output_scale);
      red_lvl = fmin(255, torque_scale);
      //green_lvl = fmin(255, 255.-torque_scale);
      blue_lvl = fmin(255, 255-torque_scale);

      //NVGcolor color1 = nvgRGBA(          red_lvl,            green_lvl,  0, 255); 
      NVGcolor color1 = nvgRGBA(          red_lvl,      0,           blue_lvl, 255); 
      //NVGcolor color2 = nvgRGBA((int)(0.5*red_lvl), (int)(0.5*green_lvl), 0, 50);
      NVGcolor color2 = nvgRGBA((int)(0.50*red_lvl), 0, (int)(0.50*blue_lvl), 50);
      track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
        color1, color2 );        
    }
   // LOGW("ui_draw_track mps=%d  cnt=%d  ov=%d  %d,%d", is_mpc, pvd->cnt, s->scene.kegman.steerOverride, red_lvl, green_lvl);
  } else {
    // Draw white vision track => blue bg
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
        nvgRGBA(0, 100, 255, 255), nvgRGBA(0, 100, 255, 50));
        //nvgRGBA(0, 255, 0, 255), nvgRGBA(0, 255, 0, 150));
  }

  /* THIS IS THE STANDARD MPC -wirelessnet2
  if (is_mpc) {
    // Draw colored MPC track
    const uint8_t *clr = bg_colors[s->status];
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
      nvgRGBA(clr[0], clr[1], clr[2], 255), nvgRGBA(clr[0], clr[1], clr[2], 255/2));
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
      COLOR_WHITE, COLOR_WHITE_ALPHA(0));
  }
  */
  nvgFillPaint(s->vg, track_bg);
  nvgFill(s->vg);
}


static void ui_draw_track_right(UIState *s, bool is_mpc, track_vertices_data *pvd) 
{
  int  nCnt = pvd->cnt; 
 if (nCnt == 0) return;

  nvgBeginPath(s->vg);
  float offset = 0;
  nvgMoveTo(s->vg, pvd->v[0].x + offset, pvd->v[0].y);
  for (int i=1; i<nCnt; i++) {
    if (pvd->v[i].y < pvd->v[i-1].y) offset = 100;
    nvgLineTo(s->vg, pvd->v[i].x + offset, pvd->v[i].y);
  }
  nvgClosePath(s->vg);

  NVGpaint track_bg;
  if (is_mpc) {
    // Draw colored MPC track
    //const uint8_t *clr = bg_colors[s->status];
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
        nvgRGBA(155, 0, 0, 255), nvgRGBA(55, 0, 0, 50));
  } else {
    // Draw white vision track => blue
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
      COLOR_WHITE, COLOR_WHITE_ALPHA(0));
  }
  nvgFillPaint(s->vg, track_bg);
  nvgFill(s->vg);
}

static void ui_draw_track_left(UIState *s, bool is_mpc, track_vertices_data *pvd) 
{
   int  nCnt = pvd->cnt;  
 if ( nCnt == 0) return;

  nvgBeginPath(s->vg);
  float offset = 0;
  nvgMoveTo(s->vg, pvd->v[0].x + offset, pvd->v[0].y);
  for (int i=1; i<nCnt; i++) {
    if (pvd->v[i].y < pvd->v[i-1].y) offset = -100;
    nvgLineTo(s->vg, pvd->v[i].x + offset, pvd->v[i].y);
  }
  nvgClosePath(s->vg);

  NVGpaint track_bg;
  if (is_mpc) {
    // Draw colored MPC track
    //const uint8_t *clr = bg_colors[s->status];
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
        nvgRGBA(155, 0, 0, 255), nvgRGBA(55, 0, 0, 50));
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
      COLOR_WHITE, COLOR_WHITE_ALPHA(0));
  }
  nvgFillPaint(s->vg, track_bg);
  nvgFill(s->vg);
}

static void draw_steering(UIState *s, float curvature) {
  float points[50];
  for (int i = 0; i < 50; i++) {
    float y_actual = i * tan(asin(clamp(i * curvature, -0.999, 0.999)) / 2.);
    points[i] = y_actual;
  }

  // ui_draw_lane_edge(s, points, 0.0, nvgRGBA(0, 0, 255, 128), 5);
}

static void draw_frame(UIState *s) {
  const UIScene *scene = &s->scene;

  if (s->scene.frontview) {
    glBindVertexArray(s->frame_vao[1]);
  } else {
    glBindVertexArray(s->frame_vao[0]);
  }

  mat4 *out_mat;
  if (s->scene.frontview || s->scene.fullview) {
    out_mat = &s->front_frame_mat;
  } else {
    out_mat = &s->rear_frame_mat;
  }
  glActiveTexture(GL_TEXTURE0);
  if (s->scene.frontview && s->cur_vision_front_idx >= 0) {
    glBindTexture(GL_TEXTURE_2D, s->frame_front_texs[s->cur_vision_front_idx]);
  } else if (!scene->frontview && s->cur_vision_idx >= 0) {
    glBindTexture(GL_TEXTURE_2D, s->frame_texs[s->cur_vision_idx]);
    #ifndef QCOM
      // TODO: a better way to do this?
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1164, 874, 0, GL_RGB, GL_UNSIGNED_BYTE, s->priv_hnds[s->cur_vision_idx]);
    #endif
  }

  glUseProgram(s->frame_program);
  glUniform1i(s->frame_texture_loc, 0);
  glUniformMatrix4fv(s->frame_transform_loc, 1, GL_TRUE, out_mat->v);

  assert(glGetError() == GL_NO_ERROR);
  glEnableVertexAttribArray(0);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, (const void*)0);
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
}

static inline bool valid_frame_pt(UIState *s, float x, float y) {
  return x >= 0 && x <= s->rgb_width && y >= 0 && y <= s->rgb_height;

}

static void update_lane_line_data(UIState *s, const float *points, float off, bool is_ghost, model_path_vertices_data *pvd) {
  pvd->cnt = 0;
  for (int i = 0; i < MODEL_PATH_MAX_VERTICES_CNT / 2; i++) {
    float px = (float)i;
    float py = points[i] - off;
    const vec4 p_car_space = (vec4){{px, py, 0., 1.}};
    const vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);
    if(!valid_frame_pt(s, p_full_frame.v[0], p_full_frame.v[1]))
      continue;
    pvd->v[pvd->cnt].x = p_full_frame.v[0];
    pvd->v[pvd->cnt].y = p_full_frame.v[1];
    pvd->cnt += 1;
  }
  for (int i = MODEL_PATH_MAX_VERTICES_CNT / 2; i > 0; i--) {
    float px = (float)i;
    float py = is_ghost?(points[i]-off):(points[i]+off);
    const vec4 p_car_space = (vec4){{px, py, 0., 1.}};
    const vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);
    if(!valid_frame_pt(s, p_full_frame.v[0], p_full_frame.v[1]))
      continue;
    pvd->v[pvd->cnt].x = p_full_frame.v[0];
    pvd->v[pvd->cnt].y = p_full_frame.v[1];
    pvd->cnt += 1;
  }
}

static void update_all_lane_lines_data(UIState *s, const PathData &path, model_path_vertices_data *pstart) {
  update_lane_line_data(s, path.points, 0.025*path.prob, false, pstart);
  float var = fmin(path.std, 0.7);
  update_lane_line_data(s, path.points, -var, true, pstart + 1);
  update_lane_line_data(s, path.points, var, true, pstart + 2);
}

static void ui_draw_lane(UIState *s, const PathData *path, model_path_vertices_data *pstart, NVGcolor color) {
  // ui_draw_lane_line(s, pstart, nvgRGBA(0, 0, 200, 200));
  ui_draw_lane_line(s, pstart, color);  
  float var = fmin(path->std, 0.7);
  color.a /= 4;
  ui_draw_lane_line(s, pstart + 1, color);
  ui_draw_lane_line(s, pstart + 2, color);  
  // ui_draw_lane_line(s, pstart + 1, nvgRGBA(0, 128, 255, 200)); //(0, 0, 200, 100)); //
  // ui_draw_lane_line(s, pstart + 2, nvgRGBA(0, 128, 255, 200)); //(0, 0, 200, 100)); //
}

static void ui_draw_vision_lanes(UIState *s) {
  const UIScene *scene = &s->scene;
  model_path_vertices_data *pvd = &s->model_path_vertices[0];
  if(s->model_changed) {
    update_all_lane_lines_data(s, scene->model.left_lane, pvd);
    update_all_lane_lines_data(s, scene->model.right_lane, pvd + MODEL_LANE_PATH_CNT);
    s->model_changed = false;
  }

  //float  left_lane =  fmax( 0.1, scene->model.left_lane.prob );
  //float  right_lane =  fmax( 0.1, scene->model.right_lane.prob );
  float  left_lane =  fmax( 0.8, scene->model.left_lane.prob );
  float  right_lane =  fmax( 0.8, scene->model.right_lane.prob );
  
  NVGcolor colorLeft = nvgRGBAf(0.2, 0.2, 1.0, left_lane );
  NVGcolor colorRight = nvgRGBAf(0.2, 0.2, 1.0, right_lane );

  if( scene->leftBlinker )
  {
    if( scene->leftBlindspot )
      colorLeft  = nvgRGBAf(0.7, 0.1, 0.1, left_lane );
    else
      colorLeft  = nvgRGBAf(0.1, 0.7, 0.1, left_lane );    
  }
  else if( scene->leftBlindspot )
  {
    colorLeft  = nvgRGBAf(0.7, 0.1, 0.1, left_lane );
  }

  if( scene->rightBlinker )
  {
    if( scene->rightBlindspot )
        colorRight  = nvgRGBAf(0.7, 0.1, 0.1, right_lane );
    else
        colorRight  = nvgRGBAf(0.1, 0.7, 0.1, right_lane ); 
  }
  else if( scene->rightBlindspot )
  {
    colorRight  = nvgRGBAf(0.7, 0.1, 0.1, right_lane );
  }


  if( scene->model.right_lane.prob < 0.6 )
  {
      if ( scene->model.right_lane.prob < 0.2 )
          colorRight = nvgRGBAf(0.7, 0.1, 0.1, right_lane );
      else if ( scene->model.right_lane.prob < 0.4 )
          colorRight = nvgRGBAf(0.7, 0.7, 0.1, right_lane );
      else 
          colorRight = nvgRGBAf(0.1, 0.7, 0.1, right_lane);      
  }
  if( scene->model.left_lane.prob < 0.5 )
  {
      if ( scene->model.left_lane.prob < 0.2 )
          colorLeft = nvgRGBAf(0.7, 0.1, 0.1, left_lane );
      else if ( scene->model.left_lane.prob < 0.4 )
          colorLeft = nvgRGBAf(0.7, 0.7, 0.1, left_lane );
      else 
          colorLeft = nvgRGBAf(0.1, 0.7, 0.1, left_lane );      
  }


//  if( scene->nTimer & 0x01 )
//  {
//     colorLeft = nvgRGBAf(0.5, 0.5, 0.5, left_lane );
//     colorRight = nvgRGBAf(0.5, 0.5, 0.5, right_lane );
//  }


  // Draw left lane edge
  ui_draw_lane(
      s, &scene->model.left_lane,
      pvd,
      colorLeft );

  // Draw right lane edge
  ui_draw_lane(
      s, &scene->model.right_lane,
      pvd + MODEL_LANE_PATH_CNT,
      colorRight );

  if( s->livempc_or_radarstate_changed ) {
    update_all_track_data(s);
    //s->livempc_or_radarstate_changed = 0;
  }

  // Draw vision path
  ui_draw_track(s, false, &s->track_vertices[0]);
  if (scene->engaged) {
    // Draw MPC path when engaged
    ui_draw_track(s, true, &s->track_vertices[1]);    
    if (scene->rightBlindspot){
      ui_draw_track_right(s, true, &s->track_vertices[1]);
    }
    if (scene->leftBlindspot){
      ui_draw_track_left(s, true, &s->track_vertices[1]);
    }      
  }
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  const UIScene *scene = &s->scene;
  if (!scene->world_objects_visible) {
    return;
  }

  const int inner_height = viz_w*9/16;
  const int ui_viz_rx = scene->ui_viz_rx;
  const int ui_viz_rw = scene->ui_viz_rw;
  const int ui_viz_ro = scene->ui_viz_ro;

  nvgSave(s->vg);
  nvgScissor(s->vg, ui_viz_rx, box_y, ui_viz_rw, box_h);

  nvgTranslate(s->vg, ui_viz_rx+ui_viz_ro, box_y + (box_h-inner_height)/2.0);
  nvgScale(s->vg, (float)viz_w / s->fb_w, (float)inner_height / s->fb_h);
  nvgTranslate(s->vg, 240.0f, 0.0);
  nvgTranslate(s->vg, -1440.0f / 2, -1080.0f / 2);
  nvgScale(s->vg, 2.0, 2.0);
  nvgScale(s->vg, 1440.0f / s->rgb_width, 1080.0f / s->rgb_height);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lanes(s);

  if (scene->lead_status) {
    draw_lead(s, scene->lead_d_rel, scene->lead_v_rel, scene->lead_y_rel);
  }
  if ((scene->lead_status2) && (std::abs(scene->lead_d_rel - scene->lead_d_rel2) > 3.0)) {
    draw_lead(s, scene->lead_d_rel2, scene->lead_v_rel2, scene->lead_y_rel2);
  }
  nvgRestore(s->vg);
}

static void ui_draw_vision_maxspeed(UIState *s) {
  /*if (!s->longitudinal_control){
    return;
  }*/
  char maxspeed_str[32];
  float maxspeed = s->scene.v_cruise;
  int maxspeed_calc = maxspeed * 0.6225 + 0.5;
  float speedlimit = s->scene.speedlimit;
  int speedlim_calc = speedlimit * 2.2369363 + 0.5;
  int speed_lim_off = s->speed_lim_off * 2.2369363 + 0.5;
  if (s->is_metric) {
    maxspeed_calc = maxspeed + 0.5;
    speedlim_calc = speedlimit * 3.6 + 0.5;
    speed_lim_off = s->speed_lim_off * 3.6 + 0.5;
  }

  bool is_cruise_set = (maxspeed != 0 && maxspeed != SET_SPEED_NA);
  bool is_speedlim_valid = s->scene.speedlimit_valid;
  bool is_set_over_limit = is_speedlim_valid && s->scene.engaged &&
                       is_cruise_set && maxspeed_calc > (speedlim_calc + speed_lim_off);

  int viz_maxspeed_w = 218;
  int viz_maxspeed_h = 202;
  int viz_maxspeed_x = (s->scene.ui_viz_rx + (bdr_s*2));
  int viz_maxspeed_y = (box_y + (bdr_s*1.5));
  int viz_maxspeed_xo = 180;

#ifdef SHOW_SPEEDLIMIT
  viz_maxspeed_w += viz_maxspeed_xo;
  viz_maxspeed_x += viz_maxspeed_w - (viz_maxspeed_xo * 2);
#else
  viz_maxspeed_xo = 0;
#endif

  // Draw Background
  ui_draw_rect(s->vg, viz_maxspeed_x, viz_maxspeed_y, viz_maxspeed_w, viz_maxspeed_h,
               is_set_over_limit ? nvgRGBA(218, 111, 37, 180) : COLOR_BLACK_ALPHA(100), 30);

  // Draw Border
  NVGcolor color = COLOR_WHITE_ALPHA(100);
  if (is_set_over_limit) {
    color = COLOR_OCHRE;
  } else if (is_speedlim_valid) {
    color = s->is_ego_over_limit ? COLOR_WHITE_ALPHA(20) : COLOR_WHITE;
  }

  ui_draw_rect(s->vg, viz_maxspeed_x, viz_maxspeed_y, viz_maxspeed_w, viz_maxspeed_h, color, 20, 10);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  const int text_x = viz_maxspeed_x + (viz_maxspeed_xo / 2) + (viz_maxspeed_w / 2);
  ui_draw_text(s->vg, text_x, 148, "MAX", 26 * 2.5, COLOR_WHITE_ALPHA(is_cruise_set ? 200 : 100), s->font_sans_regular);


   


  if (is_cruise_set) {
 
    snprintf(maxspeed_str, sizeof(maxspeed_str), "%d", maxspeed_calc);
    ui_draw_text(s->vg, text_x, 242, maxspeed_str, 42 * 2.5, COLOR_WHITE, s->font_sans_bold);
  } else {
    ui_draw_text(s->vg, text_x, 242, "-", 42 * 2.5, COLOR_WHITE_ALPHA(100), s->font_sans_semibold);
  }
 
}

static void ui_draw_vision_speedlimit(UIState *s) {
  char speedlim_str[32];
  float speedlimit = s->scene.speedlimit;
  int speedlim_calc = speedlimit * 2.2369363 + 0.5;
  if (s->is_metric) {
    speedlim_calc = speedlimit * 3.6 + 0.5;
  }

  bool is_speedlim_valid = s->scene.speedlimit_valid;
  float hysteresis_offset = 0.5;
  if (s->is_ego_over_limit) {
    hysteresis_offset = 0.0;
  }
  s->is_ego_over_limit = is_speedlim_valid && s->scene.v_ego > (speedlimit + s->speed_lim_off + hysteresis_offset);

  int viz_speedlim_w = 180;
  int viz_speedlim_h = 202;
  int viz_speedlim_x = (s->scene.ui_viz_rx + (bdr_s*2));
  int viz_speedlim_y = (box_y + (bdr_s*1.5));
  if (!is_speedlim_valid) {
    viz_speedlim_w -= 5;
    viz_speedlim_h -= 10;
    viz_speedlim_x += 9;
    viz_speedlim_y += 5;
  }
  // Draw Background
  NVGcolor color = COLOR_WHITE_ALPHA(100);
  if (is_speedlim_valid && s->is_ego_over_limit) {
    color = nvgRGBA(218, 111, 37, 180);
  } else if (is_speedlim_valid) {
    color = COLOR_WHITE;
  }
  ui_draw_rect(s->vg, viz_speedlim_x, viz_speedlim_y, viz_speedlim_w, viz_speedlim_h, color, is_speedlim_valid ? 30 : 15);

  // Draw Border
  if (is_speedlim_valid) {
    ui_draw_rect(s->vg, viz_speedlim_x, viz_speedlim_y, viz_speedlim_w, viz_speedlim_h,
                 s->is_ego_over_limit ? COLOR_OCHRE : COLOR_WHITE, 20, 10);
  }
  const float text_x = viz_speedlim_x + viz_speedlim_w / 2;
  const float text_y = viz_speedlim_y + (is_speedlim_valid ? 50 : 45);
  // Draw "Speed Limit" Text
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  color = is_speedlim_valid && s->is_ego_over_limit ? COLOR_WHITE : COLOR_BLACK;
  ui_draw_text(s->vg, text_x + (is_speedlim_valid ? 6 : 0), text_y, "SMART", 50, color, s->font_sans_semibold);
  ui_draw_text(s->vg, text_x + (is_speedlim_valid ? 6 : 0), text_y + 40, "SPEED", 50, color, s->font_sans_semibold);

  // Draw Speed Text
  color = s->is_ego_over_limit ? COLOR_WHITE : COLOR_BLACK;
  if (is_speedlim_valid) {
    snprintf(speedlim_str, sizeof(speedlim_str), "%d", speedlim_calc);
    ui_draw_text(s->vg, text_x, viz_speedlim_y + (is_speedlim_valid ? 170 : 165), speedlim_str, 42*2.3, color, s->font_sans_bold);
  } else {
    ui_draw_text(s->vg, text_x, viz_speedlim_y + (is_speedlim_valid ? 170 : 165), "N/A", 42*2.3, color, s->font_sans_semibold);
  }
}




static void ui_draw_debug(UIState *s) 
{
  UIScene &scene = s->scene;
  int ui_viz_rx = scene.ui_viz_rx;
  int ui_viz_rw = scene.ui_viz_rw;

  char str_msg[512];

  const int viz_speed_w = 280;
  const int viz_speed_x = ui_viz_rx+((ui_viz_rw/2)-(viz_speed_w/2));

  int  y_pos = 0;
  int  x_pos = 0; 

  

  nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BASELINE);
  nvgFontSize(s->vg, 36*1.5*fFontSize);


  ui_print( s, ui_viz_rx+10, 50, "S:%d",  s->awake_timeout );

  x_pos = ui_viz_rx + 300;
  y_pos = 100; 
  //ui_print( s, x_pos, y_pos+0, "B:%d,%.5f", scene.steerOverride, scene.output_scale );
  //#ui_print( s, x_pos, y_pos+150, "blindspot L:%d, R:%d", scene.leftBlindspot, scene.rightBlindspot  );
  //ui_print( s, x_pos, y_pos+150, "L1:%d, %.1f,%.1f,%.1f", (int)scene.lead_status, scene.lead_d_rel, scene.lead_y_rel , scene.lead_v_rel  );
  //ui_print( s, x_pos, y_pos+200, "L2:%d, %.1f,%.1f,%.1f", (int)scene.lead_status2, scene.lead_d_rel2, scene.lead_y_rel2 , scene.lead_v_rel2  );
  //ui_print( s, x_pos, y_pos+250, "Wheel:%.1f,%.1f,%.1f,%.1f", scene.wheel.fl, scene.wheel.fr, scene.wheel.rl, scene.wheel.rr );
  //ui_print( s, x_pos, y_pos+0, "%d, %d, %d, %d, %d", scene.params.nOpkrUIBrightness, scene.params.nLightSensor, scene.params.nSmoothBrightness, scene.params.nOpkrUIVolumeBoost, scene.params.nOpkrAutoLanechangedelay );

  
     
  //ui_print( s, x_pos, y_pos+0, "cO:%.3f  %d, %d",scene.carParams.lateralsRatom.cameraOffset, scene.cruiseState.cruiseSwState, s->livempc_or_radarstate_changed );
  // ui_print( s, x_pos, y_pos+0, "model_sum:%.1f" , scene.model_sum);
  // ui_print( s, x_pos, y_pos+50, "prob:%.2f, %.2f", scene.pathPlan.lProb, scene.pathPlan.rProb );
  // ui_print( s, x_pos, y_pos+100, "Poly:%.2f, %.2f", scene.pathPlan.lPoly, scene.pathPlan.rPoly );
  // ui_print( s, x_pos, y_pos+150, "sR:%.2f, %.2f", scene.liveParams.steerRatio, scene.pathPlan.steerRatio );
  // ui_print( s, x_pos, y_pos+200, "aO:%.2f, %.2f", scene.liveParams.angleOffset, scene.pathPlan.angleOffset );
  // ui_print( s, x_pos, y_pos+250, "aOa:%.2f", scene.liveParams.angleOffsetAverage );
  // ui_print( s, x_pos, y_pos+300, "LW:%.2f", scene.pathPlan.laneWidth );
  // ui_print( s, x_pos, y_pos+350, "aD:%.2f", scene.pathPlan.steerActuatorDelay );
  // ui_print( s, x_pos, y_pos+400, "sF:%.2f", scene.liveParams.stiffnessFactor );

  ui_print( s, x_pos, y_pos+0, "aOffset:%.2f, %.2f", scene.liveParams.angleOffset, scene.pathPlan.angleOffset );
  ui_print( s, x_pos, y_pos+50, "aOffsetAvg:%.2f", scene.liveParams.angleOffsetAverage );
  ui_print( s, x_pos, y_pos+100, "model_sum:%.1f" , scene.model_sum);
  ui_print( s, x_pos, y_pos+150, "sR:%.2f, %.2f", scene.liveParams.steerRatio, scene.pathPlan.steerRatio );
  ui_print( s, x_pos, y_pos+200, "aDelay:%.2f", scene.pathPlan.steerActuatorDelay );
  ui_print( s, x_pos, y_pos+250, "stF:%.2f", scene.liveParams.stiffnessFactor );

  ui_print( s, x_pos+530, y_pos+700, "차선폭");
  ui_print( s, x_pos+550, y_pos+750, "%.2f", scene.pathPlan.laneWidth ); 
  ui_print( s, x_pos+270, y_pos+800, "   %.2f            차선간격            %.2f", scene.pathPlan.lPoly, scene.pathPlan.rPoly );
  ui_print( s, x_pos+270, y_pos+850, "%.2f             차선인식률             %.2f", scene.pathPlan.lProb, scene.pathPlan.rProb );

  //ui_print( s, x_pos, y_pos+400, "awareness:%.2f" , scene.awareness_status);

  ui_print( s, 0, 1020, "%s", scene.alert.text1 );
  ui_print( s, 0, 1078, "%s", scene.alert.text2 );


  if( scene.params.nOpkrAccelProfile == 0 )  return;
  NVGcolor nColor = COLOR_WHITE;
  x_pos = viz_speed_x + 300;
  y_pos = 120;

  nvgFontSize(s->vg, 30);
  switch( scene.params.nOpkrAccelProfile  )
  {
    case 1: strcpy( str_msg, "1.느리게" ); nColor = nvgRGBA(100, 100, 255, 255); break;
    case 2: strcpy( str_msg, "2.보통" );    nColor = nvgRGBA(0, 180, 100, 255);  break;
    case 3: strcpy( str_msg, "3.빠르게" );  nColor = nvgRGBA(255, 100, 100, 255);  break;
    default :  sprintf( str_msg, "%d", scene.params.nOpkrAccelProfile ); nColor = COLOR_WHITE;  break;
  }
  nvgFillColor(s->vg, nColor);
  ui_print( s, x_pos, y_pos+0, "%s", str_msg );

  nvgFontSize(s->vg, 80);
  switch( scene.cruiseState.modeSel  )
  {
    case 0: strcpy( str_msg, "0.OP+ASCC" ); nColor = COLOR_WHITE; break;
    case 1: strcpy( str_msg, "1.OP+Curv." );    nColor = nvgRGBA(0, 180, 100, 255);  break;
    case 2: strcpy( str_msg, "2.OP+Dist." );  nColor = nvgRGBA(0, 150, 100, 255);  break;
    case 3: strcpy( str_msg, "3.Stock" );  nColor = nvgRGBA(200, 255, 255, 255);  break;
    case 4: strcpy( str_msg, "4.CURVATURE" );   nColor = nvgRGBA(200, 255, 255, 255);  break;    
    default :  sprintf( str_msg, "%d.NORMAL", scene.cruiseState.modeSel ); nColor = COLOR_WHITE;  break;
  }
  nvgFillColor(s->vg, nColor);  
  ui_print( s, x_pos, y_pos+80, str_msg );  
}


/*
  park @1;
  drive @2;
  neutral @3;
  reverse @4;
  sport @5;
  low @6;
  brake @7;
  eco @8;
*/
static void ui_draw_gear( UIState *s )
{
  UIScene &scene = s->scene;  
  NVGcolor nColor = COLOR_WHITE;

  int  ngetGearShifter = int(scene.getGearShifter);
  int  x_pos = 1700;
  int  y_pos = 200;
  char str_msg[512];

  nvgFontSize(s->vg, 150 );
  switch( ngetGearShifter )
  {
    case 1 : strcpy( str_msg, "P" ); nColor = nvgRGBA(200, 200, 255, 255); break;
    case 2 : strcpy( str_msg, "D" ); nColor = nvgRGBA(200, 200, 255, 255); break;
    case 3 : strcpy( str_msg, "N" ); nColor = COLOR_WHITE; break;
    case 4 : strcpy( str_msg, "R" ); nColor = COLOR_RED;  break;
    case 7 : strcpy( str_msg, "B" ); break;
    default: sprintf( str_msg, "%d", ngetGearShifter ); break;
  }

  nvgFillColor(s->vg, nColor);
  ui_print( s, x_pos, y_pos, str_msg );
}


static void ui_draw_vision_speed(UIState *s)
{
  const UIScene *scene = &s->scene;
  float speed = s->scene.v_ego * 2.2369363 + 0.5;


  if (s->is_metric){
    speed = s->scene.v_ego * 3.6 + 0.5;
  }
  const int viz_speed_w = 280;
  const int viz_speed_x = scene->ui_viz_rx+((scene->ui_viz_rw/2)-(viz_speed_w/2));
  char speed_str[32];
  NVGcolor val_color = COLOR_WHITE;

  if( scene->brakePress  ) val_color = COLOR_RED;
  else if( scene->brakeLights ) val_color = nvgRGBA(201, 34, 49, 100);

  nvgBeginPath(s->vg);
  nvgRect(s->vg, viz_speed_x, box_y, viz_speed_w, header_h);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  snprintf(speed_str, sizeof(speed_str), "%d", (int)speed);
  ui_draw_text(s->vg, viz_speed_x + viz_speed_w / 2, 240, speed_str, 96*2.5, val_color, s->font_sans_bold);
  ui_draw_text(s->vg, viz_speed_x + viz_speed_w / 2, 320, s->is_metric?"km/h":"mph", 36*2.5, COLOR_WHITE_ALPHA(200), s->font_sans_regular);

  ui_draw_debug( s );
}


static void ui_draw_vision_event(UIState *s) 
{
  const int viz_event_w = 220;
  const int viz_event_x = ((s->scene.ui_viz_rx + s->scene.ui_viz_rw) - (viz_event_w + (bdr_s*2)));
  const int viz_event_y = (box_y + (bdr_s*1.5));

  if (s->scene.decel_for_model && s->scene.engaged)
  {
    // draw winding road sign
    const int img_turn_size = 160*1.5;
    ui_draw_image(s->vg, viz_event_x - (img_turn_size / 4), viz_event_y + bdr_s - 25, img_turn_size, img_turn_size, s->img_turn, 1.0f);
  } else {
    // draw steering wheel
    const int bg_wheel_size = 96;
    const int bg_wheel_x = viz_event_x + (viz_event_w-bg_wheel_size);
    const int bg_wheel_y = viz_event_y + (bg_wheel_size/2);
    bool is_engageable = s->scene.engageable;
    NVGcolor color = COLOR_BLACK_ALPHA(0);


    if (s->status == STATUS_ENGAGED) {
      color = nvgRGBA(23, 134, 68, 255); //
    } else if (s->status == STATUS_WARNING) {
      color = COLOR_OCHRE;
    } else if (is_engageable) {
      color = nvgRGBA(23, 51, 73, 255);
    } else {
      color = nvgRGBA(100, 100, 100, 50);
    }

    if( is_engageable )  // debug_atom
    {
      ui_draw_circle_image(s->vg, bg_wheel_x, bg_wheel_y, bg_wheel_size, s->img_wheel, color, 1.0f, s->scene.angleSteers );// bg_wheel_y - 25);
    }
    else  // debug
    {
      ui_draw_gear( s );
    }
  }
}

static void ui_draw_vision_map(UIState *s) 
{
  const int map_size = 96;
  const int map_x = (s->scene.ui_viz_rx + (map_size * 3) + (bdr_s * 3));
  const int map_y = (footer_y + ((footer_h - map_size) / 2));
  ui_draw_circle_image(s->vg, map_x, map_y, map_size, s->img_map, s->scene.map_valid);
}

static void ui_draw_vision_face(UIState *s) 
{
  const int face_size = 96;
  const int face_x = (s->scene.ui_viz_rx + face_size + (bdr_s * 2));
  const int face_y = (footer_y + ((footer_h - face_size) / 2));
  ui_draw_circle_image(s->vg, face_x, face_y, face_size, s->img_face, s->scene.monitoring_active);
}

static void ui_draw_driver_view(UIState *s) 
{
  const UIScene *scene = &s->scene;
  s->scene.uilayout_sidebarcollapsed = true;
  const int frame_x = scene->ui_viz_rx;
  const int frame_w = scene->ui_viz_rw;
  const int valid_frame_w = 4 * box_h / 3;
  const int valid_frame_x = frame_x + (frame_w - valid_frame_w) / 2 + ff_xoffset;

  // blackout
  if (!scene->is_rhd) {
    NVGpaint gradient = nvgLinearGradient(s->vg, valid_frame_x + valid_frame_w,
                          box_y,
                          valid_frame_x + box_h / 2, box_y,
                          nvgRGBAf(0,0,0,1), nvgRGBAf(0,0,0,0));
    ui_draw_rect(s->vg, valid_frame_x + box_h / 2, box_y, valid_frame_w - box_h / 2, box_h, gradient);
  } else {
    NVGpaint gradient = nvgLinearGradient(s->vg, valid_frame_x,
                          box_y,
                          valid_frame_w - box_h / 2, box_y,
                          nvgRGBAf(0,0,0,1), nvgRGBAf(0,0,0,0));
    ui_draw_rect(s->vg, valid_frame_x, box_y, valid_frame_w - box_h / 2, box_h, gradient);
  }
  ui_draw_rect(s->vg, scene->is_rhd ? valid_frame_x : valid_frame_x + box_h / 2, box_y, valid_frame_w - box_h / 2, box_h, COLOR_BLACK_ALPHA(144));

  // borders
  ui_draw_rect(s->vg, frame_x, box_y, valid_frame_x - frame_x, box_h, nvgRGBA(23, 51, 161, 255));
  ui_draw_rect(s->vg, valid_frame_x + valid_frame_w, box_y, frame_w - valid_frame_w - (valid_frame_x - frame_x), box_h, nvgRGBA(23, 51, 161, 255));

  // draw face box
  if (scene->face_prob > 0.4) {
    int fbox_x;
    int fbox_y = box_y + (scene->face_y + 0.5) * box_h - 0.5 * 0.6 * box_h / 2;;
    if (!scene->is_rhd) {
      fbox_x = valid_frame_x + (1 - (scene->face_x + 0.5)) * (box_h / 2) - 0.5 * 0.6 * box_h / 2;
    } else {
      fbox_x = valid_frame_x + valid_frame_w - box_h / 2 + (scene->face_x + 0.5) * (box_h / 2) - 0.5 * 0.6 * box_h / 2;
    }
    if (std::abs(scene->face_x) <= 0.35 && std::abs(scene->face_y) <= 0.4) {
      ui_draw_rect(s->vg, fbox_x, fbox_y, 0.6 * box_h / 2, 0.6 * box_h / 2,
                   nvgRGBAf(1.0, 1.0, 1.0, 0.8 - ((std::abs(scene->face_x) > std::abs(scene->face_y) ? std::abs(scene->face_x) : std::abs(scene->face_y))) * 0.6 / 0.375),
                   35, 10);
    } else {
      ui_draw_rect(s->vg, fbox_x, fbox_y, 0.6 * box_h / 2, 0.6 * box_h / 2, nvgRGBAf(1.0, 1.0, 1.0, 0.2), 35, 10);
    }
  } else {
    ;
  }

  // draw face icon
  const int face_size = 85;
  const int face_x = (valid_frame_x + face_size + (bdr_s * 2)) + (scene->is_rhd ? valid_frame_w - box_h / 2:0);
  const int face_y = (box_y + box_h - face_size - bdr_s - (bdr_s * 1.5));
  ui_draw_circle_image(s->vg, face_x, face_y, face_size, s->img_face, scene->face_prob > 0.4);
}


//BB START: functions added for the display of various items
static int bb_ui_draw_measure(UIState *s,  const char* bb_value, const char* bb_uom, const char* bb_label,
    int bb_x, int bb_y, int bb_uom_dx,
    NVGcolor bb_valueColor, NVGcolor bb_labelColor, NVGcolor bb_uomColor,
    int bb_valueFontSize, int bb_labelFontSize, int bb_uomFontSize )  {
  const UIScene *scene = &s->scene;
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  int dx = 0;
  if (strlen(bb_uom) > 0) {
    dx = (int)(bb_uomFontSize*2.5/2);
   }
  //print value
  nvgFontFace(s->vg, "sans-bold");
  nvgFontSize(s->vg, bb_valueFontSize*2*fFontSize);
  nvgFillColor(s->vg, bb_valueColor);
  nvgText(s->vg, bb_x-dx/2, bb_y+ (int)(bb_valueFontSize*2.0)+0, bb_value, NULL);
  //print label
  nvgFontFace(s->vg, "sans-regular");
  nvgFontSize(s->vg, bb_labelFontSize*2*fFontSize);
  nvgFillColor(s->vg, bb_labelColor);
  nvgText(s->vg, bb_x, bb_y + (int)(bb_valueFontSize*2.0)+1 + (int)(bb_labelFontSize*2.0)+1, bb_label, NULL);
  //print uom
  if (strlen(bb_uom) > 0) {
      nvgSave(s->vg);
    int rx =bb_x + bb_uom_dx + bb_valueFontSize -20;
    int ry = bb_y + (int)(bb_valueFontSize*2.0/2)+18;
    nvgTranslate(s->vg,rx,ry);
    nvgRotate(s->vg, -1.5708); //-90deg in radians
    nvgFontFace(s->vg, "sans-regular");
    nvgFontSize(s->vg, (int)(bb_uomFontSize*2*fFontSize));
    nvgFillColor(s->vg, bb_uomColor);
    nvgText(s->vg, 0, 0, bb_uom, NULL);
    nvgRestore(s->vg);
  }
  return (int)((bb_valueFontSize + bb_labelFontSize)*2.0) + 0;
}


static void bb_ui_draw_measures_right(UIState *s, int bb_x, int bb_y, int bb_w ) 
{
  const UIScene *scene = &s->scene;
  int bb_rx = bb_x + (int)(bb_w/2);
  int bb_ry = bb_y;
  int bb_h = 1;
  NVGcolor lab_color = nvgRGBA(255, 255, 255, 200);
  NVGcolor uom_color = nvgRGBA(255, 255, 255, 200);
  int value_fontSize=45;
  int label_fontSize=18;
  int uom_fontSize = 15;
  int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.0) ;

  //add CPU temperature
  if( true ) 
  {
        char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
      if((int)(scene->maxCpuTemp/10) > 80) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if((int)(scene->maxCpuTemp/10) > 92) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // temp is alway in C * 10
      snprintf(val_str, sizeof(val_str), "%d", (int)(scene->maxCpuTemp/10));
      snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "CPU TEMP",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

   //add battery temperature
  if( true )
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if((int)(scene->maxBatTemp/1000) > 40) {
      val_color = nvgRGBA(255, 188, 3, 200);
    }
    if((int)(scene->maxBatTemp/1000) > 50) {
      val_color = nvgRGBA(255, 0, 0, 200);
    }
    // temp is alway in C * 1000
    snprintf(val_str, sizeof(val_str), "%d", (int)(scene->maxBatTemp/1000));
    snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "BAT TEMP",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //finally draw the frame
  bb_h += 20;
  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_h, 20);
  nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
  nvgStrokeWidth(s->vg, 6);
  nvgStroke(s->vg);
}


static void bb_ui_draw_measures_left(UIState *s, int bb_x, int bb_y, int bb_w ) 
{
  const UIScene *scene = &s->scene;
  int bb_rx = bb_x + (int)(bb_w/2);
  int bb_ry = bb_y;
  int bb_h = 1;
  NVGcolor lab_color = nvgRGBA(255, 255, 255, 200);
  NVGcolor uom_color = nvgRGBA(255, 255, 255, 200);
  int value_fontSize=45;
  int label_fontSize=18;
  int uom_fontSize = 15;
  int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.0) ;

  //add visual radar relative distance
  if( true )
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if (scene->lead_status) {
      //show RED if less than 5 meters
      //show orange if less than 15 meters
      if((int)(scene->lead_d_rel) < 15) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if((int)(scene->lead_d_rel) < 5) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // lead car relative distance is always in meters
      snprintf(val_str, sizeof(val_str), "%d", (int)scene->lead_d_rel);
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    snprintf(uom_str, sizeof(uom_str), "m   ");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "앞차거리",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //add visual radar relative speed
  if( true )
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if (scene->lead_status) {
      //show Orange if negative speed (approaching)
      //show Orange if negative speed faster than 5mph (approaching fast)
      if((int)(scene->lead_v_rel) < 0) {
        val_color = nvgRGBA(255, 188, 3, 200);
      }
      if((int)(scene->lead_v_rel) < -5) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // lead car relative speed is always in meters
      if (s->is_metric) {
         snprintf(val_str, sizeof(val_str), "%d", (int)(scene->lead_v_rel * 3.6 + 0.5));
      } else {
         snprintf(val_str, sizeof(val_str), "%d", (int)(scene->lead_v_rel * 2.2374144 + 0.5));
      }
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    if (s->is_metric) {
      snprintf(uom_str, sizeof(uom_str), "km/h");;
    } else {
      snprintf(uom_str, sizeof(uom_str), "mph");
    }
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "상대속도",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //add  steering angle
  if( true )
  {
    char val_str[16];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(0, 255, 0, 200);
      //show Orange if more than 30 degrees
      //show red if  more than 50 degrees
      if(((int)(scene->angleSteers) < -30) || ((int)(scene->angleSteers) > 30)) {
        val_color = nvgRGBA(255, 175, 3, 200);
      }
      if(((int)(scene->angleSteers) < -55) || ((int)(scene->angleSteers) > 55)) {
        val_color = nvgRGBA(255, 0, 0, 200);
      }
      // steering is in degrees
      snprintf(val_str, sizeof(val_str), "%.1f",(scene->angleSteers));

      snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "현재조향각",
        bb_rx, bb_ry, bb_uom_dx,
        val_color, lab_color, uom_color,
        value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }

  //add  desired steering angle
  if( true )
  {
    char val_str[50];
    char uom_str[6];
    NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
    if( scene->engaged ) {
      //show Orange if more than 6 degrees
      //show red if  more than 12 degrees
      if(((int)(scene->angleSteersDes) < -30) || ((int)(scene->angleSteersDes) > 30)) 
      {
        val_color = nvgRGBA(255, 255, 255, 200);
      }
      if( ((int)(scene->angleSteersDes) < -50) || ((int)(scene->angleSteersDes) > 50) ) 
      {
        val_color = nvgRGBA(255, 255, 255, 200);
      }
      // steering is in degrees
      snprintf(val_str, sizeof(val_str), "%.1f",(scene->angleSteersDes));
    } else {
       snprintf(val_str, sizeof(val_str), "-");
    }
    snprintf(uom_str, sizeof(uom_str), "");
    bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "필요조향각",
      bb_rx, bb_ry, bb_uom_dx,
      val_color, lab_color, uom_color,
      value_fontSize, label_fontSize, uom_fontSize );
    bb_ry = bb_y + bb_h;
  }


  //finally draw the frame
  bb_h += 20;
  nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_h, 20);
    nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
}


static void bb_ui_draw_UI(UIState *s)
{
  const UIScene *scene = &s->scene;
  const int bb_dml_w = 220;
  const int bb_dml_x = (scene->ui_viz_rx + (bdr_is * 2));
  const int bb_dml_y = (box_y + (bdr_is * 1.5)) + 220;

  const int bb_dmr_w = 220;
  const int bb_dmr_x = scene->ui_viz_rx + scene->ui_viz_rw - bb_dmr_w - (bdr_is * 2);
  const int bb_dmr_y = (box_y + (bdr_is * 1.5)) + 220;

  bb_ui_draw_measures_left(s, bb_dml_x, bb_dml_y, bb_dml_w);
  bb_ui_draw_measures_right(s, bb_dmr_x, bb_dmr_y, bb_dmr_w);
}
//BB END: functions added for the display of various items



static void ui_draw_vision_header(UIState *s) 
{
  const UIScene *scene = &s->scene;
  int ui_viz_rx = scene->ui_viz_rx;
  int ui_viz_rw = scene->ui_viz_rw;

  NVGpaint gradient = nvgLinearGradient(s->vg, ui_viz_rx,
                        (box_y+(header_h-(header_h/2.5))),
                        ui_viz_rx, box_y+header_h,
                        nvgRGBAf(0,0,0,0.45), nvgRGBAf(0,0,0,0));
  ui_draw_rect(s->vg, ui_viz_rx, box_y, ui_viz_rw, header_h, gradient);

  ui_draw_vision_maxspeed(s);

#ifdef SHOW_SPEEDLIMIT
  ui_draw_vision_speedlimit(s);
#endif
  ui_draw_vision_speed(s);
  ui_draw_vision_event(s);

   //BB START: add new measures panel  const int bb_dml_w = 180;
  bb_ui_draw_UI(s);
  //BB END: add new measures panel     
}


static void ui_draw_vision_footer(UIState *s) 
{
  nvgBeginPath(s->vg);
  nvgRect(s->vg, s->scene.ui_viz_rx, footer_y, s->scene.ui_viz_rw, footer_h);

  ui_draw_vision_face(s);

#ifdef SHOW_SPEEDLIMIT
   ui_draw_vision_map(s);
#endif
}



void ui_draw_vision_alert(UIState *s, cereal::ControlsState::AlertSize va_size, int va_color,
                          const char* va_text1, const char* va_text2) {
  static std::map<cereal::ControlsState::AlertSize, const int> alert_size_map = {
      {cereal::ControlsState::AlertSize::NONE, 0},
      {cereal::ControlsState::AlertSize::SMALL, 241},
      {cereal::ControlsState::AlertSize::MID, 390},
      {cereal::ControlsState::AlertSize::FULL, vwp_h}};

  const UIScene *scene = &s->scene;
  const bool hasSidebar = !scene->uilayout_sidebarcollapsed;
  const bool mapEnabled = scene->uilayout_mapenabled;
  bool longAlert1 = strlen(va_text1) > 15;

  const uint8_t *color = alert_colors[va_color];
  int alr_s = alert_size_map[va_size];

  const int alr_x = scene->ui_viz_rx-(mapEnabled?(hasSidebar?nav_w:(nav_ww)):0)-bdr_s;
  const int alr_w = scene->ui_viz_rw+(mapEnabled?(hasSidebar?nav_w:(nav_ww)):0)+(bdr_s*2);
  const int alr_h = alr_s+(va_size==cereal::ControlsState::AlertSize::NONE?0:bdr_s);
  const int alr_y = vwp_h-alr_h;

  ui_awake_aleat( s );
  //s->is_awake_command = true;
  ui_draw_rect(s->vg, alr_x, alr_y, alr_w, alr_h, nvgRGBA(color[0],color[1],color[2],(color[3]*s->alert_blinking_alpha)));

  NVGpaint gradient = nvgLinearGradient(s->vg, alr_x, alr_y, alr_x, alr_y+alr_h,
                        nvgRGBAf(0.0,0.0,0.0,0.05), nvgRGBAf(0.0,0.0,0.0,0.35));
  ui_draw_rect(s->vg, alr_x, alr_y, alr_w, alr_h, gradient);

  nvgFillColor(s->vg, COLOR_WHITE);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  if (va_size == cereal::ControlsState::AlertSize::SMALL) {
    ui_draw_text(s->vg, alr_x+alr_w/2, alr_y+alr_h/2+15, va_text1, 40*2.5, COLOR_WHITE, s->font_sans_semibold);
  } else if (va_size == cereal::ControlsState::AlertSize::MID) {
    ui_draw_text(s->vg, alr_x+alr_w/2, alr_y+alr_h/2-45, va_text1, 48*2.5, COLOR_WHITE, s->font_sans_bold);
    ui_draw_text(s->vg, alr_x+alr_w/2, alr_y+alr_h/2+75, va_text2, 36*2.5, COLOR_WHITE, s->font_sans_regular);
  } else if (va_size == cereal::ControlsState::AlertSize::FULL) {
    nvgFontSize(s->vg, (longAlert1?72:96)*2.5*fFontSize);
    nvgFontFaceId(s->vg, s->font_sans_bold);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, alr_x, alr_y+(longAlert1?360:420), alr_w-60, va_text1, NULL);
    nvgFontSize(s->vg, 48*2.5*fFontSize);
    nvgFontFaceId(s->vg,  s->font_sans_regular);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    nvgTextBox(s->vg, alr_x, alr_h-(longAlert1?300:360), alr_w-60, va_text2, NULL);
  }
}

static void ui_draw_vision(UIState *s) 
{
  const UIScene *scene = &s->scene;

  // Draw video frames
  glEnable(GL_SCISSOR_TEST);
  glViewport(scene->ui_viz_rx+scene->ui_viz_ro, s->fb_h-(box_y+box_h), viz_w, box_h);
  glScissor(scene->ui_viz_rx, s->fb_h-(box_y+box_h), scene->ui_viz_rw, box_h);
  draw_frame(s);
  glDisable(GL_SCISSOR_TEST);

  glViewport(0, 0, s->fb_w, s->fb_h);

  // Draw augmented elements
  if (!scene->frontview && !scene->fullview) {
    ui_draw_world(s);
  }

  // Set Speed, Current Speed, Status/Events
  if (!scene->frontview) {
    ui_draw_vision_header(s);
  } else {
    ui_draw_driver_view(s);
  }

  if (scene->alert_size != cereal::ControlsState::AlertSize::NONE) {
    // Controls Alerts
    ui_draw_vision_alert(s, scene->alert_size, s->status,
                            scene->alert_text1.c_str(), scene->alert_text2.c_str());
  } else {
    if (!scene->frontview){ui_draw_vision_footer(s);}
  }
}

static void ui_draw_background(UIState *s) 
{
  int bg_status = s->status;
  assert(bg_status < ARRAYSIZE(bg_colors));
  const uint8_t *color = bg_colors[bg_status];

  glClearColor(color[0]/256.0, color[1]/256.0, color[2]/256.0, 1.0);
  glClear(GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
}

void ui_draw(UIState *s) 
{
  ui_draw_background(s);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glViewport(0, 0, s->fb_w, s->fb_h);
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  ui_draw_sidebar(s);
  if (s->started && s->active_app == cereal::UiLayoutState::App::NONE && s->status != STATUS_STOPPED && s->vision_seen) {
      ui_draw_vision(s);
  }
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_draw_image(NVGcontext *vg, float x, float y, float w, float h, int image, float alpha)
{
  nvgBeginPath(vg);
  NVGpaint imgPaint = nvgImagePattern(vg, x, y, w, h, 0, image, alpha);
  nvgRect(vg, x, y, w, h);
  nvgFillPaint(vg, imgPaint);
  nvgFill(vg);
}

void ui_draw_rect(NVGcontext *vg, float x, float y, float w, float h, NVGcolor color, float r, int width) 
{
  nvgBeginPath(vg);
  r > 0? nvgRoundedRect(vg, x, y, w, h, r) : nvgRect(vg, x, y, w, h);
  if (width) {
    nvgStrokeColor(vg, color);
    nvgStrokeWidth(vg, width);
    nvgStroke(vg);
  } else {
    nvgFillColor(vg, color);
    nvgFill(vg);
  }
}

void ui_draw_rect(NVGcontext *vg, float x, float y, float w, float h, NVGpaint &paint, float r)
{
  nvgBeginPath(vg);
  r > 0? nvgRoundedRect(vg, x, y, w, h, r) : nvgRect(vg, x, y, w, h);
  nvgFillPaint(vg, paint);
  nvgFill(vg);
}

#ifdef NANOVG_GL3_IMPLEMENTATION
static const char frame_vertex_shader[] =
  "#version 150 core\n"
  "in vec4 aPosition;\n"
  "in vec4 aTexCoord;\n"
  "uniform mat4 uTransform;\n"
  "out vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vTexCoord = aTexCoord;\n"
  "}\n";

static const char frame_fragment_shader[] =
  "#version 150 core\n"
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "out vec4 vTexCoord;\n"
  "out vec4 outColor;\n"
  "void main() {\n"
  "  outColor = texture(uTexture, vTexCoord.xy);\n"
  "}\n";
#else
static const char frame_vertex_shader[] =
  "attribute vec4 aPosition;\n"
  "attribute vec4 aTexCoord;\n"
  "uniform mat4 uTransform;\n"
  "varying vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vTexCoord = aTexCoord;\n"
  "}\n";

static const char frame_fragment_shader[] =
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "varying vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_FragColor = texture2D(uTexture, vTexCoord.xy);\n"
  "}\n";
#endif

static const mat4 device_transform = {{
  1.0,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

// frame from 4/3 to box size with a 2x zoom
static const mat4 frame_transform = {{
  2*(4./3.)/((float)viz_w/box_h), 0.0, 0.0, 0.0,
  0.0, 2.0, 0.0, 0.0,
  0.0, 0.0, 1.0, 0.0,
  0.0, 0.0, 0.0, 1.0,
}};

// frame from 4/3 to 16/9 display
static const mat4 full_to_wide_frame_transform = {{
  .75,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

void ui_nvg_init(UIState *s) {
  // init drawing
  s->vg = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  s->font_courbd = nvgCreateFont(s->vg, "courbd", "../assets/fonts/courbd.ttf");
  assert(s->font_courbd >= 0);
  s->font_sans_regular = nvgCreateFont(s->vg, "sans-regular", "../assets/fonts/opensans_regular.ttf");
  assert(s->font_sans_regular >= 0);
  s->font_sans_semibold = nvgCreateFont(s->vg, "sans-semibold", "../assets/fonts/opensans_semibold.ttf");
  assert(s->font_sans_semibold >= 0);
  s->font_sans_bold = nvgCreateFont(s->vg, "sans-bold", "../assets/fonts/opensans_bold.ttf");
  assert(s->font_sans_bold >= 0);

  s->img_wheel = nvgCreateImage(s->vg, "../assets/img_chffr_wheel.png", 1);
  assert(s->img_wheel != 0);
  s->img_turn = nvgCreateImage(s->vg, "../assets/img_trafficSign_turn.png", 1);
  assert(s->img_turn != 0);
  s->img_face = nvgCreateImage(s->vg, "../assets/img_driver_face.png", 1);
  assert(s->img_face != 0);
  s->img_map = nvgCreateImage(s->vg, "../assets/img_map.png", 1);
  assert(s->img_map != 0);
  s->img_button_settings = nvgCreateImage(s->vg, "../assets/images/button_settings.png", 1);
  assert(s->img_button_settings != 0);
  s->img_button_home = nvgCreateImage(s->vg, "../assets/images/button_home.png", 1);
  assert(s->img_button_home != 0);
  s->img_battery = nvgCreateImage(s->vg, "../assets/images/battery.png", 1);
  assert(s->img_battery != 0);
  s->img_battery_charging = nvgCreateImage(s->vg, "../assets/images/battery_charging.png", 1);
  assert(s->img_battery_charging != 0);

  for(int i=0;i<=5;++i) {
    char network_asset[32];
    snprintf(network_asset, sizeof(network_asset), "../assets/images/network_%d.png", i);
    s->img_network[i] = nvgCreateImage(s->vg, network_asset, 1);
    assert(s->img_network[i] != 0);
  }

  // init gl
  s->frame_program = load_program(frame_vertex_shader, frame_fragment_shader);
  assert(s->frame_program);

  s->frame_pos_loc = glGetAttribLocation(s->frame_program, "aPosition");
  s->frame_texcoord_loc = glGetAttribLocation(s->frame_program, "aTexCoord");

  s->frame_texture_loc = glGetUniformLocation(s->frame_program, "uTexture");
  s->frame_transform_loc = glGetUniformLocation(s->frame_program, "uTransform");

  glViewport(0, 0, s->fb_w, s->fb_h);

  glDisable(GL_DEPTH_TEST);

  assert(glGetError() == GL_NO_ERROR);

  for(int i = 0; i < 2; i++) {
    float x1, x2, y1, y2;
    if (i == 1) {
      // flip horizontally so it looks like a mirror
      x1 = 0.0;
      x2 = 1.0;
      y1 = 1.0;
      y2 = 0.0;
    } else {
      x1 = 1.0;
      x2 = 0.0;
      y1 = 1.0;
      y2 = 0.0;
    }
    const uint8_t frame_indicies[] = {0, 1, 2, 0, 2, 3};
    const float frame_coords[4][4] = {
      {-1.0, -1.0, x2, y1}, //bl
      {-1.0,  1.0, x2, y2}, //tl
      { 1.0,  1.0, x1, y2}, //tr
      { 1.0, -1.0, x1, y1}, //br
    };

    glGenVertexArrays(1,&s->frame_vao[i]);
    glBindVertexArray(s->frame_vao[i]);
    glGenBuffers(1, &s->frame_vbo[i]);
    glBindBuffer(GL_ARRAY_BUFFER, s->frame_vbo[i]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(frame_coords), frame_coords, GL_STATIC_DRAW);
    glEnableVertexAttribArray(s->frame_pos_loc);
    glVertexAttribPointer(s->frame_pos_loc, 2, GL_FLOAT, GL_FALSE,
                          sizeof(frame_coords[0]), (const void *)0);
    glEnableVertexAttribArray(s->frame_texcoord_loc);
    glVertexAttribPointer(s->frame_texcoord_loc, 2, GL_FLOAT, GL_FALSE,
                          sizeof(frame_coords[0]), (const void *)(sizeof(float) * 2));
    glGenBuffers(1, &s->frame_ibo[i]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s->frame_ibo[i]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(frame_indicies), frame_indicies, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glBindVertexArray(0);
  }

  s->front_frame_mat = matmul(device_transform, full_to_wide_frame_transform);
  s->rear_frame_mat = matmul(device_transform, frame_transform);

  for(int i = 0;i < UI_BUF_COUNT; i++) {
    s->khr[i] = 0;
    s->priv_hnds[i] = NULL;
    s->khr_front[i] = 0;
    s->priv_hnds_front[i] = NULL;
  }
}
