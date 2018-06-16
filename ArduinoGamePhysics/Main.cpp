
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
// CArduinoSerialInput�N���X
// �ȉ���URL�Ɍf�ڂ���Ă����\�[�X�R�[�h�����ς��Ďg�p
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
    // �V���A���|�[�g�̐ڑ�
    this->mHandle = ::CreateFileA(
        static_cast<LPCSTR>(portName), GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (this->mHandle == INVALID_HANDLE_VALUE) {
        if (::GetLastError() == ERROR_FILE_NOT_FOUND)
            ::MessageBoxA(
                NULL, "Arduino�}�C�R���{�[�h���ڑ�����Ă��܂���.",
                "Error", MB_OK | MB_ICONEXCLAMATION);
        else
            ::MessageBoxA(
                NULL, "Arduino�}�C�R���{�[�h�Ɛڑ��ł��܂���ł���.",
                "Error", MB_OK | MB_ICONEXCLAMATION);

        this->mIsConnected = false;
        return;
    }

    // �V���A���|�[�g�̌��݂̃p�����[�^�̎擾
    DCB dcbSerialInputParameters;
    ZeroMemory(&dcbSerialInputParameters, sizeof(DCB));

    if (!::GetCommState(this->mHandle, &dcbSerialInputParameters)) {
        ::MessageBoxA(
            NULL, "�V���A���|�[�g�̌��݂̃p�����[�^�̎擾�Ɏ��s���܂���.",
            "Error", MB_OK | MB_ICONEXCLAMATION);
        this->mIsConnected = false;
        return;
    }

    // �V���A���|�[�g�̃p�����[�^�̐ݒ�
    dcbSerialInputParameters.BaudRate = CBR_57600;
    dcbSerialInputParameters.ByteSize = 8;
    dcbSerialInputParameters.StopBits = ONESTOPBIT;
    dcbSerialInputParameters.Parity = NOPARITY;
    dcbSerialInputParameters.fDtrControl = DTR_CONTROL_ENABLE;

    if (!::SetCommState(this->mHandle, &dcbSerialInputParameters)) {
        ::MessageBoxA(
            NULL, "�V���A���|�[�g�̃p�����[�^�̐ݒ�Ɏ��s���܂���.",
            "Error", MB_OK | MB_ICONEXCLAMATION);
        this->mIsConnected = false;
        return;
    }

    // �V���A���|�[�g�̐ڑ��̊���
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

        // ���s�R�[�h�����o������ǂݍ��݂��I����
        if (pBuffer[i] == '\n')
            return numOfBytesRead;
    }

    return 0;
}

//
// GameState�񋓑�
//
enum class GameState
{
    Start,
    Play,
    GameOver
};

//
// CGame�N���X
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
    CArduinoSerialInput * mArduinoInput;    // �V���A���|�[�g�̓���
    char* mSerialInputBuffer;               // ���̓o�b�t�@
    double mInputValue0;                    // �Z���T�̓��͒l0
    double mInputValue1;                    // �Z���T�̓��͒l1

    b2World* mWorld;                        // �V�~�����[�V�������s�����[���h
    b2Vec2 mGravity;                        // ���[���h�̏d��
    int mVelocityIterations;                // ���x�̐��m��
    int mPositionIterations;                // �ʒu�̐��m��

    b2BodyDef mGroundBodyDef;               // �n�ʂ̒�`���
    b2Body* mGroundBody;                    // �n��
    b2PolygonShape mGroundBox;              // �n�ʂ̌`��

    b2BodyDef mLineBodyDef;                 // �Ζʂ̒�`���
    b2Body* mLineBody;                      // �Ζ�
    b2EdgeShape mLineShape0;                // �Ζ�0�̌`��
    b2EdgeShape mLineShape1;                // �Ζ�1�̌`��
    b2FixtureDef mLineFixtureDef0;          // �Ζ�0�̒�`���
    b2FixtureDef mLineFixtureDef1;          // �Ζ�1�̒�`���
    b2Fixture* mLineFixture0;               // �Ζ�0
    b2Fixture* mLineFixture1;               // �Ζ�1

    b2BodyDef mCircleBodyDef0;              // ���̒�`���0
    b2BodyDef mCircleBodyDef1;              // ���̒�`���1
    b2CircleShape mCircleShape;             // ���̌`��
    b2FixtureDef mCircleFixtureDef0;        // ���̒�`���0
    b2FixtureDef mCircleFixtureDef1;        // ���̒�`���1

    int mGenerateCircleCounter;             // ���̏o���J�E���^
    int mGenerateCircleCounterThreshold;    // ���̏o���J�E���^��臒l

    static const char* ApplicationName;     // �A�v���P�[�V������
    static const int WindowWidth;           // �E�B���h�E�̉���
    static const int WindowHeight;          // �E�B���h�E�̏c��
    static const int ColorBitDepth;         // �J���[�r�b�g��
    static const int RefreshRate;           // �t���[�����[�g
    static const char* PortName;            // �ڑ��|�[�g��
    static const int SerialInputBufferSize; // ���̓o�b�t�@�̃T�C�Y
};

const char* CGame::ApplicationName = "ArduinoGamePhysics";  // �A�v���P�[�V������
const int CGame::WindowWidth = 1024;                        // �E�B���h�E�̉���
const int CGame::WindowHeight = 768;                        // �E�B���h�E�̏c��
const int CGame::ColorBitDepth = 32;                        // �J���[�r�b�g��
const int CGame::RefreshRate = 60;                          // �t���[�����[�g
const char* CGame::PortName = "\\\\.\\COM3";                // �ڑ��|�[�g��
const int CGame::SerialInputBufferSize = 256;               // ���̓o�b�t�@�̃T�C�Y

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
    // ���̓o�b�t�@�̊m��
    this->mSerialInputBuffer = new char[CGame::SerialInputBufferSize];

    if (this->mSerialInputBuffer == nullptr)
        return false;

    // �V���A���|�[�g�ڑ��̏�����
    this->mArduinoInput = new CArduinoSerialInput(CGame::PortName);

    if (this->mArduinoInput == nullptr) {
        ::MessageBoxA(
            NULL, "Arduino�}�C�R���{�[�h�Ƃ̐ڑ��Ɏ��s���܂���.",
            CGame::ApplicationName, MB_OK | MB_ICONEXCLAMATION);
        return false;
    }

    if (!this->mArduinoInput->IsConnected()) {
        ::MessageBoxA(
            NULL, "Arduino�}�C�R���{�[�h�Ƃ̐ڑ��Ɏ��s���܂���.",
            CGame::ApplicationName, MB_OK | MB_ICONEXCLAMATION);
        return false;
    }

    ::MessageBoxA(
        NULL, "Arduino�}�C�R���{�[�h�Ƃ̐ڑ����m������܂���.",
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
    // �V���A���|�[�g����f�[�^���擾
    int bytesRead = this->mArduinoInput->Read(
        this->mSerialInputBuffer, CGame::SerialInputBufferSize);

    if (!bytesRead)
        return;

    this->mSerialInputBuffer[bytesRead] = '\0';

    // �Z���T�̏o�͒l�̃f�o�b�O�o��
    // ::OutputDebugStringA(static_cast<LPCSTR>(this->mSerialInputBuffer));

    // �Z���T�̏o�͒l�̓ǂݍ���
    int inputValue0;
    int inputValue1;

    if (sscanf(this->mSerialInputBuffer, "%d,%d", &inputValue0, &inputValue1) != 2)
        return;

    // �w���d�ݕt���ړ����ς̌v�Z
    this->mInputValue0 = 0.99 * this->mInputValue0 + 0.01 * static_cast<double>(inputValue0);
    this->mInputValue1 = 0.99 * this->mInputValue1 + 0.01 * static_cast<double>(inputValue1);
}

void CGame::InitializeBox2D()
{
    // �V�~�����[�V�������s�����[���h�̍쐬
    this->mGravity.Set(0.0f, -9.8f);
    this->mWorld = new b2World(this->mGravity);
    this->mWorld->SetAllowSleeping(true);

    // ���x�ƈʒu�̐��m���̌���
    this->mVelocityIterations = 10;
    this->mPositionIterations = 15;

    // �n�ʂ̍쐬
    this->mGroundBodyDef.type = b2BodyType::b2_staticBody;
    this->mGroundBodyDef.position.Set(0.0f, 0.0f);
    this->mGroundBody = this->mWorld->CreateBody(&this->mGroundBodyDef);
    this->mGroundBox.SetAsBox(32.0f, 4.0f);
    this->mGroundBody->CreateFixture(&this->mGroundBox, 0.0f);

    // �Ζʂ̍쐬
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

    // ���̏�����
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

    // �Ζ�0�̊p�x�̒���
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
    // �n�ʂ̕`��
    DxLib::DrawBoxAA(
        this->mGroundBox.m_vertices[3].x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
        -this->mGroundBox.m_vertices[3].y * 16.0f + static_cast<float>(CGame::WindowHeight),
        this->mGroundBox.m_vertices[1].x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
        -this->mGroundBox.m_vertices[1].y * 16.0f + static_cast<float>(CGame::WindowHeight),
        DxLib::GetColor(0, 0, 255), TRUE);
    
    // �Ζʂ̕`��
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

    // ���̕`��
    for (b2Body* pCircleBody = this->mWorld->GetBodyList();
        pCircleBody != nullptr;
        pCircleBody = pCircleBody->GetNext()) {
        // ���łȂ��ꍇ�̓X�L�b�v
        if (pCircleBody->GetType() != b2BodyType::b2_dynamicBody)
            continue;
        
        // ���̕`��
        DxLib::DrawCircleAA(
            pCircleBody->GetPosition().x * 16.0f + static_cast<float>(CGame::WindowWidth) / 2.0f,
            -pCircleBody->GetPosition().y * 16.0f + static_cast<float>(CGame::WindowHeight),
            this->mCircleShape.m_radius * 16.0f,
            128, DxLib::GetColor(0, 255, 0), TRUE);
    }
}

int CGame::Run()
{
    // �V���A���|�[�g�ڑ��̏�����
    this->InitializeArduinoInput();

    // Dx���C�u�����̏�����
    DxLib::ChangeWindowMode(TRUE);
    DxLib::SetGraphMode(
        CGame::WindowWidth, CGame::WindowHeight,
        CGame::ColorBitDepth, CGame::RefreshRate);
    DxLib::SetWindowSize(CGame::WindowWidth, CGame::WindowHeight);
    DxLib::SetMainWindowText(CGame::ApplicationName);
    DxLib::SetOutApplicationLogValidFlag(FALSE);

    if (DxLib::DxLib_Init() == -1)
        return -1;

    // Box2D���C�u�����̏�����
    this->InitializeBox2D();

    while (DxLib::ProcessMessage() == 0) {
        DxLib::ClearDrawScreen();
        DxLib::SetDrawScreen(DX_SCREEN_BACK);

        // �Z���T����̓d���l���擾
        this->HandleInput();

        // �X�V����
        this->Update();

        // �`�揈��
        this->Draw();

        DxLib::ScreenFlip();
    }

    // Box2D���C�u�����̏I������
    this->FinalizeBox2D();

    // Dx���C�u�����̏I������
    DxLib::DxLib_End();

    // �V���A���|�[�g�ڑ��̏I������
    this->FinalizeArduinoInput();

    return 0;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    return CGame::GetInstance()->Run();
}
