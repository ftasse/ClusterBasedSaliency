in vec3 position;
in vec3 normal;
in vec4 color;

uniform mat4 vp_tranform;
// unirform mat4 m;
// uniform mat3 m_3x3_inv_transp;

out vec4 f_color;
out vec3 f_normal;
out vec3 f_position;
out vec3 world_position;

struct lightSource
{
  vec4 position;
  vec4 diffuse;
  float constantAttenuation, linearAttenuation, quadraticAttenuation;
  float spotCutoff, spotExponent;
  vec3 spotDirection;
};

struct material
{
  vec4 diffuse;
};
// material mymaterial =  material(vec4(1.0, 0.8, 0.8, 1.0));

mat4 m = mat4(
	1.0, 0.0, 0.0, 0.0,
	0.0, 1.0, 0.0, 0.0,
	0.0, 0.0, 1.0, 0.0,
	0.0, 0.0, 0.0, 1.0
);
mat3 m_3x3_inv_transp = mat3(
	1.0, 0.0, 0.0,
	0.0, 1.0, 0.0,
	0.0, 0.0, 1.0
);
mat4 mvp = vp_tranform*m;

vec3 compute_color(lightSource light0, material mymaterial, vec4 v_coord, vec3 v_normal)
{
	vec3 normalDirection = normalize(m_3x3_inv_transp * v_normal);
	vec3 lightDirection;
	float attenuation;

  if (light0.position.w == 0.0) // directional light
  {
      attenuation = 1.0; // no attenuation
      lightDirection = normalize(vec3(light0.position));
  }
  else // point or spot light (or other kind of light)
  {
  	vec3 vertexToLightSource = vec3(light0.position - m * v_coord);
  	float distance = length(vertexToLightSource);
  	lightDirection = normalize(vertexToLightSource);
  	attenuation = 1.0 / (light0.constantAttenuation
  		+ light0.linearAttenuation * distance
  		+ light0.quadraticAttenuation * distance * distance);

      /*if (light0.spotCutoff <= 90.0) // spotlight
      {
      	float clampedCosine = max(0.0, dot(-lightDirection, normalize(light0.spotDirection)));
		  if (clampedCosine < cos(light0.spotCutoff * 3.14159 / 180.0)) // outside of spotlight cone
		  {
		  	attenuation = 0.0;
		  }
		  else
		  {
		  	attenuation = attenuation * pow(clampedCosine, light0.spotExponent);
		  }
		}*/
	}
	vec3 diffuseReflection = attenuation
    * vec3(light0.diffuse) * vec3(mymaterial.diffuse)
	  * max(0.0, dot(normalDirection, lightDirection));

	return diffuseReflection;
}

void main(void) {
  gl_Position = mvp * vec4(position, 1.0);

  lightSource light0 = lightSource(
    vec4(0.0, 1.0, 1.0, 1.0),
    vec4(1.0, 1.0, 1.0, 1.0),
    0.0, 1.0, 0.0,
    80.0, 20.0,
    vec3(-1.0, -0.5, -1.0)
  );

  f_color = vec4(compute_color(light0, material(vec4(125.0/255, 125.0/255, 125.0/255, 1)), vec4(position, 1.0), -normal), color[3]);
  f_normal = normal;
  f_position = vec3(gl_Position);
  world_position = (m * vec4(position, 1.0)).xyz;
}