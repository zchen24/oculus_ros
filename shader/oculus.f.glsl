#version 400


// -------------------------------------
// Example Shader from Oculus


//"Texture2D Texture   : register(t0);                                                    \n"
//"SamplerState Linear : register(s0);                                                    \n"

//"float4 main(in float4 oPosition  : SV_Position, in float4 oColor : COLOR,              \n"
//"            in float2 oTexCoord0 : TEXCOORD0,   in float2 oTexCoord1 : TEXCOORD1,      \n"
//"            in float2 oTexCoord2 : TEXCOORD2)   : SV_Target                            \n"
//"{                                                                                      \n"
//// 3 samples for fixing chromatic aberrations
//"    float ResultR = Texture.Sample(Linear, oTexCoord0.xy).r;                           \n"
//"    float ResultG = Texture.Sample(Linear, oTexCoord1.xy).g;                           \n"
//"    float ResultB = Texture.Sample(Linear, oTexCoord2.xy).b;                           \n"
//"    return float4(ResultR * oColor.r, ResultG * oColor.g, ResultB * oColor.b, 1.0);    \n"
//"}";


uniform float fadeFactor;
uniform sampler2D texture;  // texture from app (video frame)

in vec2 oTexcoord0;  // from vertex shader, vertex coordinate
in vec2 oTexcoord1;  // from vertex shader, vertex coordinate
in vec2 oTexcoord2;  // from vertex shader, vertex coordinate
in float oVignette;      // from vertex shader, fading

in vec2 oTexcoord;

void main()
{
    // sample value
    float ResultR = texture2D(texture, oTexcoord0).r * 1.0;
    float ResultG = texture2D(texture, oTexcoord1).g * 1.0;
    float ResultB = texture2D(texture, oTexcoord2).b * 1.0;

//    gl_FragColor = vec4(ResultR, ResultG, ResultB, 1.0) * fadeFactor;
    gl_FragColor = vec4(ResultR, ResultG, ResultB, 1.0) * oVignette;

//    gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0) * oVignette;
//    gl_FragColor = texture2D(texture, oTexcoord) * fadeFactor;
//    gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
}
