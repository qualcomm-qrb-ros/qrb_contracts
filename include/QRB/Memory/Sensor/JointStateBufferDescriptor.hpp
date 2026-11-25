// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_JOINT_STATE_BUFFER_DESCRIPTOR_HPP
#define QRB_JOINT_STATE_BUFFER_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace QRB
{
namespace Memory
{

/**
 * @addtogroup RoboticsMemory Robotics Memory Descriptors
 * @{
 */

/** @brief Maximum number of characters in a joint name, excluding the terminating null. */
static const std::size_t QRB_JOINT_NAME_MAX_LEN = 63U;

/**
 * @struct JointState_t
 * @brief Fixed-size state of one joint, stored in the DMA payload.
 *
 * All three measurements are present. There is no validity mask or reserved field;
 * missing measurements require an explicit agreement at the producer/consumer level.
 * Producers must construct the records in suitably aligned storage before using them.
 * Joint names longer than QRB_JOINT_NAME_MAX_LEN are not truncated: check the status
 * returned by QCString::Assign.
 *
 * @param name      Joint identifier (e.g. "left_elbow_joint").
 * @param position  Position in rad for revolute joints, or m for prismatic joints.
 * @param velocity  Velocity in rad/s for revolute joints, or m/s for prismatic joints.
 * @param effort    Effort in N*m for revolute joints, or N for prismatic joints.
 */
typedef struct alignas( 8 ) JointState
{
    QC::QCString<QRB_JOINT_NAME_MAX_LEN> name;
    double position{ 0.0 };
    double velocity{ 0.0 };
    double effort{ 0.0 };
} JointState_t;

static_assert( std::is_trivially_copyable<JointState_t>::value,
               "JointState_t must be trivially copyable for DMA buffer storage" );
static_assert( std::is_standard_layout<JointState_t>::value,
               "JointState_t must be standard layout for DMA buffer storage" );

/**
 * @struct JointStateBufferDescriptor_t
 * @brief Descriptor for one frame of joint states in a contiguous DMA payload.
 *
 * Buffer layout:
 * @code
 *   pBuf + offset -> [ JointState_t[0] | ... | JointState_t[jointCount-1] ]
 * @endcode
 *
 * The inherited size is the allocation size, offset is the payload byte offset,
 * and validSize is the valid payload length. Producers must set validSize to
 * CalcBufferSize(jointCount), and align pBuf + offset to alignof(JointState_t).
 * Copying or moving a descriptor does not deep-copy or take ownership of its payload.
 *
 * QC_BUFFER_TYPE_CUSTOM_4 is shared with other descriptors without a dedicated tag,
 * as in TFMessageDescriptor_t; the tag alone does not identify the C++ payload type.
 *
 * @param timestampNs  Frame acquisition time in nanoseconds. Producer and consumer
 *                     must agree on the clock domain before mapping to ROS Header.stamp.
 * @param frameId      Reference frame, matching ROS Header.frame_id.
 * @param jointCount   Number of valid JointState_t records.
 */
typedef struct JointStateBufferDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    JointStateBufferDescriptor()
        : QC::Memory::BufferDescriptor_t(), timestampNs( 0 ), jointCount( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_4;
    }

    ~JointStateBufferDescriptor() override = default;

    JointStateBufferDescriptor( const JointStateBufferDescriptor &other ) = default;
    JointStateBufferDescriptor( JointStateBufferDescriptor &&other ) = default;

    // Preserve the other assignment overloads of the base class.
    using QC::Memory::BufferDescriptor_t::operator=;

    /** @brief Copies base fields, preserving joint metadata for the caller to update. */
    JointStateBufferDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept override;

    /** @brief Non-owning base move: copies base fields and preserves joint metadata. */
    JointStateBufferDescriptor &operator=( QC::Memory::BufferDescriptor_t &&other ) noexcept;

    /** @brief Copies the buffer reference and all joint metadata, not the payload. */
    JointStateBufferDescriptor &operator=( const JointStateBufferDescriptor &other ) noexcept;

    /** @brief Moves the buffer reference and fixed-size joint metadata, not the payload. */
    JointStateBufferDescriptor &operator=( JointStateBufferDescriptor &&other ) noexcept;

    /** @brief Dispatches to the assignment overload matching the source descriptor type. */
    JointStateBufferDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept override;

    /** @brief Non-owning root move with the same type dispatch as root copy assignment. */
    JointStateBufferDescriptor &operator=( QC::Memory::QCBufferDescriptorBase_t &&other ) noexcept;

    /**
     * @brief Returns the indexed joint, or nullptr for an invalid index or payload.
     * @note Checks the complete frame length, allocation bounds, and data alignment.
     *       Buffer mapping and object lifetime remain the caller's responsibility.
     */
    JointState_t *GetJointPtr( uint32_t index )
    {
        return const_cast<JointState_t *>(
                static_cast<const JointStateBufferDescriptor &>( *this ).GetJointPtr( index ) );
    }

    /** @brief Read-only access with the same validation as the non-const overload. */
    const JointState_t *GetJointPtr( uint32_t index ) const
    {
        if ( index >= jointCount || pBuf == nullptr || offset > size )
        {
            return nullptr;
        }

        const size_t required = CalcBufferSize( jointCount );
        const size_t available = size - offset;
        if ( required == 0 || validSize > available || required > validSize || GetDataSize() < required )
        {
            return nullptr;
        }

        const void *data = GetDataPtr();
        if ( data == nullptr || reinterpret_cast<std::uintptr_t>( data ) % alignof( JointState_t ) != 0 )
        {
            return nullptr;
        }
        return static_cast<const JointState_t *>( data ) + index;
    }

    /** @brief Payload bytes required for count joints; 0 for zero count or overflow. */
    static size_t CalcBufferSize( uint32_t count )
    {
        const size_t elementCount = static_cast<size_t>( count );
        if ( elementCount > std::numeric_limits<size_t>::max() / sizeof( JointState_t ) )
        {
            return 0;
        }
        return elementCount * sizeof( JointState_t );
    }

    uint64_t  timestampNs; /**< Acquisition timestamp in the agreed clock domain (ns). */
    FrameId_t frameId;     /**< Fixed-capacity reference frame identifier. */
    uint32_t  jointCount;  /**< Number of valid joints in the DMA payload. */

} JointStateBufferDescriptor_t;

inline JointStateBufferDescriptor &JointStateBufferDescriptor::operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept
{
    QC::Memory::BufferDescriptor_t::operator=( other );
    // Some QCNode versions omit alignment from base copy assignment.
    this->alignment = other.alignment;
    this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
    return *this;
}

inline JointStateBufferDescriptor &JointStateBufferDescriptor::operator=( QC::Memory::BufferDescriptor_t &&other ) noexcept
{
    return operator=( static_cast<const QC::Memory::BufferDescriptor_t &>( other ) );
}

inline JointStateBufferDescriptor &JointStateBufferDescriptor::operator=( const JointStateBufferDescriptor &other ) noexcept
{
    if ( this != &other )
    {
        JointStateBufferDescriptor::operator=( static_cast<const QC::Memory::BufferDescriptor_t &>( other ) );
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->jointCount = other.jointCount;
    }
    return *this;
}

inline JointStateBufferDescriptor &JointStateBufferDescriptor::operator=( JointStateBufferDescriptor &&other ) noexcept
{
    // All members are non-owning or fixed-size, so move assignment is a copy.
    return operator=( static_cast<const JointStateBufferDescriptor &>( other ) );
}

inline JointStateBufferDescriptor &JointStateBufferDescriptor::operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept
{
    if ( this != &other )
    {
        const JointStateBufferDescriptor_t *jointDesc = dynamic_cast<const JointStateBufferDescriptor_t *>( &other );
        const QC::Memory::BufferDescriptor_t *bufferDesc = dynamic_cast<const QC::Memory::BufferDescriptor_t *>( &other );
        if ( jointDesc != nullptr )
        {
            JointStateBufferDescriptor::operator=( *jointDesc );
        }
        else if ( bufferDesc != nullptr )
        {
            JointStateBufferDescriptor::operator=( *bufferDesc );
        }
        else
        {
            QC::Memory::BufferDescriptor_t::operator=( other );
            this->alignment = other.alignment;
            this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
        }
    }
    return *this;
}

inline JointStateBufferDescriptor &JointStateBufferDescriptor::operator=( QC::Memory::QCBufferDescriptorBase_t &&other ) noexcept
{
    return operator=( static_cast<const QC::Memory::QCBufferDescriptorBase_t &>( other ) );
}

/** @} */  // end of RoboticsMemory

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_JOINT_STATE_BUFFER_DESCRIPTOR_HPP
