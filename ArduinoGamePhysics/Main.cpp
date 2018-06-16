
/* ArduinoGamePhysics */
/* Main.cpp */

#define NOMINMAX
#include "DxLib.h"

#include "Box2D/Box2D.h"
#pragma comment(lib, "Box2D.lib")

template <typename T>
T Pi = static_cast<T>(3.141592653589793);

template <typename T>
inline T ConvertDegreeToRadian(T degreeValue)
{
    return degreeValue * Pi<T> / static_cast<T>(180.0);
}

template <typename T>
inline T ConvertRadianToDegree(T radianValue)
{
    return radianValue * static_cast<T>(180.0) / Pi<T>;
}

//
// CArduinoSerialInputクラス
// 以下のURLに掲載されていたソースコードを改変して使用
// https://blog.manash.me/serial-communication-with-an-arduino-using-c-on-windows-d08710186498
//
class CArduinoSerialInput
{
public:
    CArduinoSerialInput(const char* portName);
    ~CArduinoSerialInput();

    int Read(char* pBuffer, unsigned int bufferSize);
    inline bool IsConnected() const { return this->mIsConnected; }

private:
    HANDLE mHandle;
    COMSTAT mCommStatus;
    DWORD mError;
    bool mIsConnected;
};

CArduinoSerialInput::CArduinoSerialInput(const char* portName)
{
    // シリアルポートの接続
    this->mHandle = ::CreateFileA(
        static_cast<LPCSTR>(portName), GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (this->mHandle == INVALID_HANDLE_VALUE) {
        if (::GetLastError() == ERROR_FILE_NOT_FOUND)
            ::MessageBoxA(
                NULL, "Arduinoマイコンボードが接続されていません.",
                "Error", MB_OK | MB_ICONEXCLAMATION);
        else
            ::MessageBoxA(
                NULL, "Arduinoマイコンボードと接続できませんでした.",
                "Error", MB_OK | MB_ICONEXCLAMATION);

        this->mIsConnected = false;
        return;
    }

    // シリアルポートの現在のパラメータの取得
    DCB dcbSerialInputParameters;
    ZeroMemory(&dcbSerialInputParameters, sizeof(DCB));

    if (!::GetCommState(this->mHandle, &dcbSerialInputParameters)) {
        ::MessageBoxA(
            NULL, "シリアルポートの現在のパラメータの取得に失敗しました.",
            "Error", MB_OK | MB_ICONEXCLAMATION);
        this->mIsConnected = false;
        return;
    }

    // シリアルポートのパラメータの設定
    dcbSerialInputParameters.BaudRate = CBR_57600;
    dcbSerialInputParameters.ByteSize = 8;
    dcbSerialInputParameters.StopBits = ONESTOPBIT;
    dcbSerialInputParameters.Parity = NOPARITY;
    dcbSerialInputParameters.fDtrControl = DTR_CONTROL_ENABLE;

    if (!::SetCommState(this->mHandle, &dcbSerialInputParameters)) {
        ::MessageBoxA(
            NULL, "シリアルポートのパラメータの設定に失敗しました.",
            "Error", MB_OK | MB_ICONEXCLAMATION);
        this->mIsConnected = false;
        return;
    }

    // シリアルポートの接続の完了
    this->mIsConnected = true;
    ::PurgeComm(this->mHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    ::Sleep(20);
}

CArduinoSerialInput::~CArduinoSerialInput()
{
    if (this->mIsConnected) {
        ::CloseHandle(this->mHandle);
        this->mIsConnected = false;
    }
}

int CArduinoSerialInput::Read(char* pBuffer, unsigned int bufferSize)
{
    DWORD numOfBytesRead = 0;
    DWORD numOfBytesToRead = 0;
    DWORD bytesRead;

    ::ClearCommError(this->mHandle, &this->mError, &this->mCommStatus);

    if (this->mCommStatus.cbInQue > 0)
        if (this->mCommStatus.cbInQue > bufferSize)
            numOfBytesToRead = bufferSize;
        else
            numOfBytesToRead = this->mCommStatus.cbInQue;

    for (DWORD i = 0; i < numOfBytesToRead; ++i) {
        if (!::ReadFile(this->mHandle, &pBuffer[i], 1, &bytesRead, NULL))
            return numOfBytesRead;

        ++numOfBytesRead;

        // 改行コードを検出したら読み込みを終える
        if (pBuffer[i] == '\n')
            return numOfBytesRead;
    }

    return 0;
}

//
// GameState列挙体
//
enum class GameState
{
    Start,
    Play,
    GameOver
};

//
// CGameクラス
//
class CGame final
{
public:
    static CGame* GetInstance();

    CGame(const CGame&) = delete;
    CGame(CGame&&) = delete;
    CGame& operator=(const CGame&) = delete;
    CGame& operator=(CGame&&) = delete;

    int Run();

private:
    CGame();
    ~CGame() = default;

    bool InitializeArduinoInput();
    void FinalizeArduinoInput();
    void HandleInput();
    void InitializeBox2D();
    void FinalizeBox2D();
    void Update();
    void Draw();

private:
    CArduinoSerialInput * mArduinoInput;    // シリアルポートの入力
    char* mSerialInputBuffer;               // 入力バッファ
    double mInputValue0;                    // センサの入力値0
    double mInputValue1;                    // センサの入力値1

    b2World* mWorld;                        // シミュレーションを行うワールド
    b2Vec2 mGravity;                        // ワールドの重力
    int mVelocityIterations;                // 速度の正確さ
    int mPositionIterations;                // 位置の正確さ

    b2BodyDef mGroundBodyDef;               // 地面の定義情報
    b2Body* mGroundBody;                    // 地面
    b2PolygonShape mGroundBox;              // 地面の形状

    b2BodyDef mLineBodyDef;                 // 斜面の定義情報
    b2Body* mLineBody;                      // 斜面
    b2EdgeShape mLineShape0;                // 斜面0の形状
    b2EdgeShape mLineShape1;                // 斜面1の形状
    b2FixtureDef mLineFixtureDef0;          // 斜面0の定義情報
    b2FixtureDef mLineFixtureDef1;          // 斜面1の定義情報
    b2Fixture* mLineFixture0;               // 斜面0
    b2Fixture* mLineFixture1;               // 斜面1

    b2BodyDef mCircleBodyDef0;              // 球の定義情報0
    b2BodyDef mCircleBodyDef1;              // 球の定義情報1
    b2CircleShape mCircleShape;             // 球の形状
    b2FixtureDef mCircleFixtureDef0;        // 球の定義情報0
    b2FixtureDef mCircleFixtureDef1;        // 球の定義情報1

    int mGenerateCircleCounter;             // 球の出現カウンタ
    int mGenerateCircleCounterThreshold;    // 球の出現カウンタの閾値

    static const char* ApplicationName;     // アプリケーション名
    static const int WindowWidth;           // ウィンドウの横幅
    static const int WindowHeight;          // ウィンドウの縦幅
    static const int ColorBitDepth;         // カラービット数
    static const int RefreshRate;           // フレームレート
    static const char* PortName;            // 接続ポート名
    static const int SerialInputBufferSize; // 入力バッファのサイズ
};

const char* CGame::ApplicationName = "ArduinoGamePhysics";  // アプリケーション名
const int CGame::WindowWidth = 1024;                        // ウィンドウの横幅
const int CGame::WindowHeight = 768;                        // ウィンドウの縦幅
const int CGame::ColorBitDepth = 32;                        // カラービット数
const int CGame::RefreshRate = 60;                          // フレームレート
const char* CGame::PortName = "\\\\.\\COM3";                // 接続ポート名
const int CGame::SerialInputBufferSize = 256;               // 入力バッファのサイズ

CGame* CGame::GetInstance()
{
    static CGame theInstance;
    return &theInstance;
}

CGame::CGame() :
    mArduinoInput(nullptr),
    mSerialInputBuffer(nullptr),
    mInputValue0(0.0),
    mInputValue1(0.0),
    mWorld(nullptr),
    mGravity(),
    mVelocityIterations(0),
    mPositionIterations(0),
    mGroundBodyDef(),
    mGroundBody(nullptr),
    mGroundBox(),
    mLineBodyDef(),
    mLineBody(nullptr),
    mLineShape0(),
    mLineShape1(),
    mLineFixtureDef0(),
    mLineFixtureDef1(),
    mLineFixture0(nullptr),
    mLineFixture1(nullptr),
    mCircleBodyDef0(),
    mCircleBodyDef1(),
    mCircleShape(),
    mCircleFixtureDef0(),
    mCircleFixtureDef1(),
    mGenerateCircleCounter(0),
    mGenerateCircleCounterThreshold(48)
{
}

bool CGame::InitializeArduinoInput()
{
    // 入力バッファの確保
    this->mSerialInputBuffer = new char[CGame::SerialInputBufferSize];

    if (this->mSerialInputBuffer == nullptr)
        return false;

    // シリアルポート接続の初期化
    this->mArduinoInput = new CArduinoSerialInput(CGame::PortName);

    if (this->mArduinoInput == nullptr) {
        ::MessageBoxA(
            NULL, "Arduinoマイコンボードとの接続に失敗しました.",
            CGame::ApplicationName, MB_OK | MB_ICONEXCLAMATION);
        return false;
    }

    if (!this->mArduinoInput->IsConnected()) {
        ::MessageBoxA(
            NULL, "Arduinoマイコンボードとの接続に失敗しました.",
            CGame::ApplicationName, MB_OK | MB_ICONEXCLAMATION);
        return false;
    }

    ::MessageBoxA(
        NULL, "Arduinoマイコンボードとの接続が確立されました.",
        CGame::ApplicationName, MB_OK | MB_ICONINFORMATION);

    return true;
}

void CGame::FinalizeArduinoInput()
{
    if (this->mSerialInputBuffer != nullptr) {
        delete[] this->mSerialInputBuffer;
        this->mSerialInputBuffer = nullptr;
    }

    if (this->mArduinoInput != nullptr) {
        delete this->mArduinoInput;
        this->mArduinoInput = nullptr;
    }
}

void CGame::HandleInput()
{
    // シリアルポートからデータを取得
    int bytesRead = this->mArduinoInput->Read(
        this->mSerialInputBuffer, CGame::SerialInputBufferSize);

    if (!bytesRead)
        return;

    this->mSerialInputBuffer[bytesRead] = '\0';

    // センサの出力値のデバッグ出力
    // ::OutputDebugStringA(static_cast<LPCSTR>(this->mSerialInputBuffer));

    // センサの出力値の読み込み
    int inputValue0;
    int inputValue1;

    if (sscanf(this->mSerialInputBuffer, "%d,%d", &inputValue0, &inputValue1) != 2)
        return;

    // 指数重み付き移動平均の計算
    this->mInputValue0 = 0.99 * this->mInputValue0 + 0.01 * static_cast<double>(inputValue0);
    this->mInputValue1 = 0.99 * this->mInputValue1 + 0.01 * static_cast<double>(inputValue1);
}

void CGame::InitializeBox2D()
{
    // シミュレーションを行うワールドの作成
    this->mGravity.Set(0.0f, -9.8f);
    this->mWorld = new b2World(this->mGravity);
    this->mWorld->SetAllowSleeping(true);

    // 速度と位置の正確さの決定
    this->mVelocityIterations = 10;
    this->mPositionIterations = 15;

    // 地面の作成
    this->mGroundBodyDef.type = b2BodyType::b2_staticBody;
    this->mGroundBodyDef.position.Set(0.0f, 0.0f);
    this->mGroundBody = this->mWorld->CreateBody(&this->mGroundBodyDef);
    this->mGroundBox.SetAsBox(32.0f, 4.0f);
    this->mGroundBody->CreateFixture(&this->mGroundBox, 0.0f);

    // 斜面の作成
    this->mLineBodyDef.type = b2BodyType::b2_staticBody;
    this->mLineBodyDef.position.Set(0.0f, 0.0f);
    this->mLineBody = this->mWorld->CreateBody(&this->mLineBodyDef);

    this->mLineShape0.Set(b2Vec2(2.0f, 18.0f), b2Vec2(25.0f, 26.0f));
    this->mLineFixtureDef0.shape = &this->mLineShape0;
    this->mLineFixtureDef0.density = 0.0f;
    this->mLineFixture0 = this->mLineBody->CreateFixture(&this->mLineFixtureDef0);

    this->mLineShape1.Set(b2Vec2(-25.0f, 16.0f), b2Vec2(-2.0f, 8.0f));
    this->mLineFixtureDef1.shape = &this->mLineShape1;
    this->mLineFixtureDef1.density = 0.0f;
    this->mLineFixture1 = this->mLineBody->CreateFixture(&this->mLineFixtureDef1);

    // 球の初期化
    this->mCircleBodyDef0.type = b2BodyType::b2_dynamicBody;
    this->mCircleBodyDef0.position.Set(20.0f, 45.0f);
    this->mCircleBodyDef1.type = b2BodyType::b2_dynamicBody;
    this->mCircleBodyDef1.position.Set(-20.0f, 45.0f);

    this->mCircleShape.m_radius = 1.0f;

    this->mCircleFixtureDef0.shape = &this->mCircleShape;
    this->mCircleFixtureDef0.density = 0.05f;
    this->mCircleFixtureDef0.friction = 0.3f;
    this->mCircleFixtureDef0.restitution = 0.5f;

    this->mCircleFixtureDef1.shape = &this->mCircleShape;
    this->mCircleFixtureDef1.density = 1.0f;
    this->mCircleFixtureDef1.friction = 0.3f;
    this->mCircleFixtureDef1.restitution = 0.1f;
}

void CGame::FinalizeBox2D()
{
    if (this->mWorld != nullptr) {
        delete this->mWorld;
        this->mWorld = nullptr;
    }
}

void CGame::Update()
{
    this->mGenerateCircleCounter++;

    if (this->mGenerateCircleCounter > this->mGenerateCircleCounterThreshold) {
        this->mGenerateCircleCounter = 0;
        b2Body* pCircleBody = this->mWorld->CreateBody(&this->mCircleBodyDef0);
        pCircleBody->CreateFixture(&this->mCircleFixtureDef0);
        pCircleBody = this->mWorld->CreateBody(&this->mCircleBodyDef1);
        pCircleBody->CreateFixture(&this->mCircleFixtureDef1);
    }

    // 斜面0の角度の調節
    b2EdgeShape* pLineShape = dynamic_cast<b2EdgeShape*>(this->mLineFixture0->GetShape());
    float lineTheta = -static_cast<float>((this->mInputValue0 - this->mInputValue1) / 1000.0f);
    pLineShape->m_vertex1.x = 11.5f - 11.5f * cos(lineTheta);
    pLineShape->m_vertex1.y = 22.0f + 11.5f * sin(lineTheta);
    pLineShape->m_vertex2.x = 11.5f + 11.5f * cos(lineTheta);
    pLineShape->m_vertex2.y = 22.0f - 11.5f * sin(lineTheta);

    this->mWorld->Step(1.0f / 30.0f, this->mVelocityIterations, this->mPositionIterations);
}

void CGame::Draw()
{
    // 地面の描画
    DxLib::DrawBoxAA(
        this->mGroundBox.m_vertices[3].x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
        -this->mGroundBox.m_vertices[3].y * 16.0f + static_cast<float>(CGame::WindowHeight),
        this->mGroundBox.m_vertices[1].x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
        -this->mGroundBox.m_vertices[1].y * 16.0f + static_cast<float>(CGame::WindowHeight),
        DxLib::GetColor(0, 0, 255), TRUE);
    
    // 斜面の描画
    b2EdgeShape* pLineShape = dynamic_cast<b2EdgeShape*>(this->mLineFixture0->GetShape());

    DxLib::DrawLineAA(
        pLineShape->m_vertex1.x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
        -pLineShape->m_vertex1.y * 16.0f + static_cast<float>(CGame::WindowHeight),
        pLineShape->m_vertex2.x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
        -pLineShape->m_vertex2.y * 16.0f + static_cast<float>(CGame::WindowHeight),
        DxLib::GetColor(255, 0, 0), 3.0f);

    pLineShape = dynamic_cast<b2EdgeShape*>(this->mLineFixture1->GetShape());

    DxLib::DrawLineAA(
        pLineShape->m_vertex1.x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
        -pLineShape->m_vertex1.y * 16.0f + static_cast<float>(CGame::WindowHeight),
        pLineShape->m_vertex2.x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
        -pLineShape->m_vertex2.y * 16.0f + static_cast<float>(CGame::WindowHeight),
        DxLib::GetColor(255, 0, 0), 3.0f);

    // 球の描画
    for (b2Body* pCircleBody = this->mWorld->GetBodyList();
        pCircleBody != nullptr;
        pCircleBody = pCircleBody->GetNext()) {
        // 球でない場合はスキップ
        if (pCircleBody->GetType() != b2BodyType::b2_dynamicBody)
            continue;
        
        // 球の描画
        DxLib::DrawCircleAA(
            pCircleBody->GetPosition().x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
            -pCircleBody->GetPosition().y * 16.0f + static_cast<float>(CGame::WindowHeight),
            this->mCircleShape.m_radius * 16.0f,
            128, DxLib::GetColor(0, 255, 0), TRUE);
    }
}

int CGame::Run()
{
    // シリアルポート接続の初期化
    this->InitializeArduinoInput();

    // Dxライブラリの初期化
    DxLib::ChangeWindowMode(TRUE);
    DxLib::SetGraphMode(
        CGame::WindowWidth, CGame::WindowHeight,
        CGame::ColorBitDepth, CGame::RefreshRate);
    DxLib::SetWindowSize(CGame::WindowWidth, CGame::WindowHeight);
    DxLib::SetMainWindowText(CGame::ApplicationName);
    DxLib::SetOutApplicationLogValidFlag(FALSE);

    if (DxLib::DxLib_Init() == -1)
        return -1;

    // Box2Dライブラリの初期化
    this->InitializeBox2D();

    while (DxLib::ProcessMessage() == 0) {
        DxLib::ClearDrawScreen();
        DxLib::SetDrawScreen(DX_SCREEN_BACK);

        // センサからの電圧値を取得
        this->HandleInput();

        // 更新処理
        this->Update();

        // 描画処理
        this->Draw();

        DxLib::ScreenFlip();
    }

    // Box2Dライブラリの終了処理
    this->FinalizeBox2D();

    // Dxライブラリの終了処理
    DxLib::DxLib_End();

    // シリアルポート接続の終了処理
    this->FinalizeArduinoInput();

    return 0;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    return CGame::GetInstance()->Run();
}
