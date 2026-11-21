// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_IMU_DESCRIPTOR_HPP
#define QRB_IMU_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>

namespace QRB
{
namespace Memory
{

/**
 * @struct ImuDescriptor
 * @brief Descriptor for a robotics IMU buffer holding exactly one IMU sample.
 *
 * Extends @c QC::Memory::BufferDescriptor_t with the metadata and measurement
 * fields needed to store a single IMU reading. The layout mirrors ROS 2
 * @c sensor_msgs/Imu (excluding @c header).
 *
 * Inherited Members from QC::Memory::BufferDescriptor_t:
 * @param name          Human-readable buffer name.
 * @param pBuf          Virtual address of the DMA buffer.
 * @param size          Total allocated size of the DMA buffer (bytes).
 * @param type          Buffer type tag (@c QC_BUFFER_TYPE_CUSTOM_0 for IMU).
 * @param alignment     Alignment of the buffer.
 * @param cache         Cache attributes.
 * @param allocatorType Allocator used to allocate the buffer.
 * @param dmaHandle     DMA handle of the buffer.
 * @param pid           Process ID of the allocating process.
 * @param validSize     Number of bytes of valid IMU data currently stored.
 * @param offset        Byte offset of the valid data within the DMA buffer.
 * @param id            User-assigned identifier to distinguish the buffer.
 *
 * New Members:
 * @param timestampNs                     Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC),
 *                                        matching ROS 2 std_msgs/Header.stamp.
 * @param frameId                         Coordinate frame of the IMU (e.g. "imu_link"),
 *                                        matching ROS 2 std_msgs/Header.frame_id.
 * @param orientation                     Orientation quaternion (x, y, z, w).
 * @param orientation_covariance          Row-major 3x3 covariance about x, y, z axes.
 * @param angular_velocity                Angular velocity (x, y, z) in rad/s.
 * @param angular_velocity_covariance     Row-major 3x3 covariance about x, y, z axes.
 * @param linear_acceleration             Linear acceleration (x, y, z) in m/s^2.
 * @param linear_acceleration_covariance  Row-major 3x3 covariance about x, y, z axes.
 */
typedef struct ImuDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    ImuDescriptor() :
        QC::Memory::BufferDescriptor_t(),
        timestampNs( 0 ),
        orientation(),
        angular_velocity(),
        linear_acceleration()
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_0;
    }

    ~ImuDescriptor() override = default;

    ImuDescriptor( const ImuDescriptor &other ) = default;

    // Inherit the other assignment overloads of the base class, as qcnode descriptors do.
    using QC::Memory::BufferDescriptor_t::operator=;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; IMU-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    ImuDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept;

    /**
     * @brief Copy-assign from another @c ImuDescriptor_t, copying every field one by one.
     */
    ImuDescriptor &operator=( const ImuDescriptor &other );

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     *
     * Dispatches to the most derived overload that @p other is an instance of.
     */
    ImuDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept;

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint64_t     timestampNs;                             /**< Acquisition timestamp (ns, CLOCK_MONOTONIC). */
    FrameId_t    frameId;                                 /**< Coordinate frame identifier (e.g. "imu_link"). */

    Quaternion_t orientation;                             /**< Orientation quaternion (x, y, z, w). */
    double       orientation_covariance[9] = { -1.0 };    /**< Row-major 3x3 covariance about x, y, z axes. Element 0 of -1 means unknown. */

    Vector3d_t   angular_velocity;                        /**< Angular velocity (x, y, z) in rad/s. */
    double       angular_velocity_covariance[9] = { -1.0 };  /**< Row-major 3x3 covariance about x, y, z axes. Element 0 of -1 means unknown. */

    Vector3d_t   linear_acceleration;                     /**< Linear acceleration (x, y, z) in m/s^2. */
    double       linear_acceleration_covariance[9] = { -1.0 };  /**< Row-major 3x3 covariance about x, y, z axes. Element 0 of -1 means unknown. */

} ImuDescriptor_t;

inline ImuDescriptor &ImuDescriptor::operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept
{
    QC::Memory::BufferDescriptor_t::operator=( other );
    this->type = QC::QC_BUFFER_TYPE_CUSTOM_0;
    return *this;
}

inline ImuDescriptor &ImuDescriptor::operator=( const ImuDescriptor &other )
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( other );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_0;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->orientation = other.orientation;
        (void) std::copy( std::begin( other.orientation_covariance ), std::end( other.orientation_covariance ),
                          std::begin( this->orientation_covariance ) );
        this->angular_velocity = other.angular_velocity;
        (void) std::copy( std::begin( other.angular_velocity_covariance ),
                          std::end( other.angular_velocity_covariance ),
                          std::begin( this->angular_velocity_covariance ) );
        this->linear_acceleration = other.linear_acceleration;
        (void) std::copy( std::begin( other.linear_acceleration_covariance ),
                          std::end( other.linear_acceleration_covariance ),
                          std::begin( this->linear_acceleration_covariance ) );
    }
    return *this;
}

inline ImuDescriptor &ImuDescriptor::operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept
{
    if ( this != &other )
    {
        const ImuDescriptor_t *pImuDesc = dynamic_cast<const ImuDescriptor_t *>( &other );
        const QC::Memory::BufferDescriptor_t *pBufDesc =
                dynamic_cast<const QC::Memory::BufferDescriptor_t *>( &other );
        if ( nullptr != pImuDesc )
        {
            ImuDescriptor::operator=( *pImuDesc );
        }
        else if ( nullptr != pBufDesc )
        {
            ImuDescriptor::operator=( *pBufDesc );
        }
        else
        {
            QC::Memory::BufferDescriptor_t::operator=( other );
        }
    }
    return *this;
}

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_IMU_DESCRIPTOR_HPP
